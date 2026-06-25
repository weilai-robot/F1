#!/usr/bin/env python3
"""
axis_check.py — 机器人坐标系轴对齐诊断脚本

对比 3 个定位源在行走过程中的运动方向:
  1. MuJoCo Ground Truth  (/mujoco/ground_truth)   — 绝对真值
  2. FastLIO2 估计        (/Odometry)               — LIO 估计
  3. Nav2 TF (AMCL 校正后)  (map→base_link)          — 导航用

用法:
  # 方式1: 手动推速度 (自动切 walk_mode + 推 vx=0.2 直线)
  python3 axis_check.py --auto

  # 方式2: 导航过程中监控 (需另起终端发 send_nav_goal)
  python3 axis_check.py --duration 30

输出:
  终端实时表格 + 最终 JSON 报告 (reports/axis_check_<ts>.json)
"""

import argparse
import json
import math
import os
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32, Float64MultiArray, Twist
from nav_msgs.msg import Odometry
import tf2_ros


GREEN  = '\033[0;32m'
YELLOW = '\033[1;33m'
RED    = '\033[0;31m'
CYAN   = '\033[0;36m'
BOLD   = '\033[1m'
NC     = '\033[0m'


class AxisChecker(Node):
    def __init__(self, duration, auto_mode):
        super().__init__('axis_checker')

        self.duration = duration
        self.auto_mode = auto_mode
        self.finished = False

        # ── 数据缓冲 ──
        self.gt_samples = deque(maxlen=5000)
        self.fl_samples = deque(maxlen=5000)
        self.tf_samples = deque(maxlen=5000)

        # ── TF Buffer ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=50,
        )

        # ── 订阅 ──
        self.create_subscription(Float64MultiArray, '/mujoco/ground_truth',
                                  self._gt_cb, sensor_qos)
        self.create_subscription(Odometry, '/Odometry',
                                  self._fl_cb, sensor_qos)

        # ── 自动模式: 发 walk_mode + cmd_vel ──
        if self.auto_mode:
            self._walk_pub = self.create_publisher(Float32, '/walk_mode', 10)
            self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_limiter', 10)
            self._auto_phase = 'wait'  # wait → walk → drive → stop
            self._auto_timer = self.create_timer(0.2, self._auto_cb)
            self._auto_start = time.monotonic()
        else:
            self._auto_phase = 'manual'

        # ── TF 采样定时器 ──
        self.create_timer(0.1, self._tf_sample)
        # ── 报告定时器 ──
        self.create_timer(5.0, self._report)
        # ── 结束定时器 ──
        self.create_timer(1.0, self._check_timeout)
        self._start_wall = time.monotonic()

        # ── 上次的值 (计算 delta) ──
        self._prev_gt = None
        self._prev_fl = None
        self._prev_tf = None

        print(f'{CYAN}AxisChecker 启动{NC}')
        print(f'  模式: {"自动 (walk_mode + vx=0.2)" if auto_mode else "手动监控"}')
        print(f'  时长: {duration}s')
        print(f'  格式: Δx=ΔX方向位移, Δy=ΔY方向位移, yaw=朝向(度)')
        print(f'{"="*80}')

    def _gt_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 7:
            return
        d = msg.data
        sim_t = d[0]
        x, y = d[1], d[2]
        yaw = d[6]
        self.gt_samples.append((sim_t, x, y, yaw))

    def _fl_cb(self, msg: Odometry):
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.fl_samples.append((sim_t, x, y, yaw))

    def _tf_sample(self):
        try:
            trans = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            t = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            self.tf_samples.append((t, x, y, yaw))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException, tf2_ros.TimeoutException):
            pass

    def _auto_cb(self):
        """自动模式: 依次 walk_mode → cmd_vel → stop"""
        elapsed = time.monotonic() - self._auto_start

        if elapsed < 5.0:
            # Phase 1: 持续发 walk_mode (5Hz)
            if int(elapsed * 5) % 5 == 0:
                msg = Float32()
                msg.data = 0.0
                self._walk_pub.publish(msg)

        elif elapsed < 8.0:
            # Phase 2: 等待行走稳定 (3s)
            if self._auto_phase != 'walk':
                print(f'{YELLOW}  walk_mode 已发送, 等待稳定...{NC}')
                self._auto_phase = 'walk'

        elif elapsed < self.duration - 2.0:
            # Phase 3: 推 vx=0.2 直线
            if self._auto_phase != 'drive':
                print(f'{GREEN}  开始推 vx=0.2 直线行走...{NC}')
                self._auto_phase = 'drive'
            msg = Twist()
            msg.linear.x = 0.2
            self._cmd_pub.publish(msg)

        elif self._auto_phase != 'stop':
            # Phase 4: 停止
            self._auto_phase = 'stop'
            msg = Twist()
            self._cmd_pub.publish(msg)
            print(f'{YELLOW}  停止推速度{NC}')

    def _report(self):
        if len(self.gt_samples) < 2 or len(self.fl_samples) < 2:
            print(f'{YELLOW}  数据不足, 等待...{NC}')
            return

        elapsed = time.monotonic() - self._start_wall

        # 取最近 3 秒的数据做 delta
        now_t = self.gt_samples[-1][0]
        cutoff = now_t - 3.0

        gt_recent = [s for s in self.gt_samples if s[0] >= cutoff]
        fl_recent = [s for s in self.fl_samples if s[0] >= cutoff]
        tf_recent = list(self.tf_samples)[-30:]  # ~3s @ 10Hz

        if len(gt_recent) < 2 or len(fl_recent) < 2:
            return

        gt0, gt1 = gt_recent[0], gt_recent[-1]
        fl0, fl1 = fl_recent[0], fl_recent[-1]

        gt_dx = gt1[1] - gt0[1]
        gt_dy = gt1[2] - gt0[2]
        gt_yaw = math.degrees(gt1[3])

        fl_dx = fl1[1] - fl0[1]
        fl_dy = fl1[2] - fl0[2]
        fl_yaw = math.degrees(fl1[3])

        tf_dx, tf_dy, tf_yaw = 0.0, 0.0, 0.0
        if len(tf_recent) >= 2:
            tf0, tf1 = tf_recent[0], tf_recent[-1]
            tf_dx = tf1[1] - tf0[1]
            tf_dy = tf1[2] - tf0[2]
            tf_yaw = math.degrees(tf1[3])

        print(f'\n{CYAN}[{elapsed:.0f}s] 最近 3s 位移对比{NC}')
        print(f'{"─"*70}')
        print(f'{"源":<16} {"Δx":>8} {"Δy":>8} {"yaw°":>8} {"方向判断"}')
        print(f'{"─"*70}')

        gt_dir = self._dir_str(gt_dx, gt_dy)
        fl_dir = self._dir_str(fl_dx, fl_dy)
        tf_dir = self._dir_str(tf_dx, tf_dy)

        print(f'{"GT (真值)":<16} {gt_dx:>+8.3f} {gt_dy:>+8.3f} {gt_yaw:>+8.1f} {gt_dir}')
        print(f'{"FastLIO2":<16} {fl_dx:>+8.3f} {fl_dy:>+8.3f} {fl_yaw:>+8.1f} {fl_dir}')
        print(f'{"Nav2 TF":<16} {tf_dx:>+8.3f} {tf_dy:>+8.3f} {tf_yaw:>+8.1f} {tf_dir}')
        print(f'{"─"*70}')

        # 一致性检查
        issues = []
        if gt_dx * fl_dx < -0.01 and (abs(gt_dx) > 0.05 or abs(fl_dx) > 0.05):
            issues.append(f'{RED}⚠ X 轴翻转: GT Δx={gt_dx:+.3f} vs FL Δx={fl_dx:+.3f}{NC}')
        if gt_dy * fl_dy < -0.01 and (abs(gt_dy) > 0.05 or abs(fl_dy) > 0.05):
            issues.append(f'{RED}⚠ Y 轴翻转: GT Δy={gt_dy:+.3f} vs FL Δy={fl_dy:+.3f}{NC}')
        if abs(gt_yaw - fl_yaw) > 90 and (abs(gt_yaw) > 10 or abs(fl_yaw) > 10):
            issues.append(f'{RED}⚠ Yaw 偏差: GT={gt_yaw:+.1f}° vs FL={fl_yaw:+.1f}°{NC}')
        if abs(gt_yaw - tf_yaw) > 90 and (abs(gt_yaw) > 10 or abs(tf_yaw) > 10):
            issues.append(f'{RED}⚠ TF Yaw 偏差: GT={gt_yaw:+.1f}° vs TF={tf_yaw:+.1f}°{NC}')

        if not issues:
            print(f'{GREEN}✓ 三源方向一致{NC}')
        else:
            for issue in issues:
                print(issue)
        print()

    def _dir_str(self, dx, dy):
        """位移方向简述"""
        if abs(dx) < 0.02 and abs(dy) < 0.02:
            return '静止'
        parts = []
        if abs(dx) > abs(dy):
            if abs(dx) > 0.02:
                parts.append('+X (东)' if dx > 0 else '-X (西)')
        else:
            if abs(dy) > 0.02:
                parts.append('+Y (北)' if dy > 0 else '-Y (南)')
        return ' '.join(parts) if parts else '微动'

    def _check_timeout(self):
        elapsed = time.monotonic() - self._start_wall
        if elapsed >= self.duration:
            self.finished = True

    def compute_final_report(self):
        """计算最终报告"""
        if len(self.gt_samples) < 3 or len(self.fl_samples) < 3:
            return {'error': 'insufficient_data'}

        # 全程统计
        gt_start = self.gt_samples[0]
        gt_end = self.gt_samples[-1]
        fl_start = self.fl_samples[0]
        fl_end = self.fl_samples[-1]

        # TF (取首尾)
        tf_start = self.tf_samples[0] if self.tf_samples else None
        tf_end = self.tf_samples[-1] if self.tf_samples else None

        report = {
            'duration_sec': round(time.monotonic() - self._start_wall, 1),
            'gt_samples': len(self.gt_samples),
            'fl_samples': len(self.fl_samples),
            'tf_samples': len(self.tf_samples),

            'gt_start': {'x': round(gt_start[1], 3), 'y': round(gt_start[2], 3),
                         'yaw_deg': round(math.degrees(gt_start[3]), 1)},
            'gt_end': {'x': round(gt_end[1], 3), 'y': round(gt_end[2], 3),
                       'yaw_deg': round(math.degrees(gt_end[3]), 1)},
            'gt_delta': {
                'dx': round(gt_end[1] - gt_start[1], 3),
                'dy': round(gt_end[2] - gt_start[2], 3),
                'dyaw_deg': round(math.degrees(gt_end[3] - gt_start[3]), 1),
            },

            'fl_start': {'x': round(fl_start[1], 3), 'y': round(fl_start[2], 3),
                         'yaw_deg': round(math.degrees(fl_start[3]), 1)},
            'fl_end': {'x': round(fl_end[1], 3), 'y': round(fl_end[2], 3),
                       'yaw_deg': round(math.degrees(fl_end[3]), 1)},
            'fl_delta': {
                'dx': round(fl_end[1] - fl_start[1], 3),
                'dy': round(fl_end[2] - fl_start[2], 3),
                'dyaw_deg': round(math.degrees(fl_end[3] - fl_start[3]), 1),
            },
        }

        # 方向一致性
        gt_d = report['gt_delta']
        fl_d = report['fl_delta']

        x_flip = (gt_d['dx'] * fl_d['dx'] < 0) and (abs(gt_d['dx']) > 0.05 or abs(fl_d['dx']) > 0.05)
        y_flip = (gt_d['dy'] * fl_d['dy'] < 0) and (abs(gt_d['dy']) > 0.05 or abs(fl_d['dy']) > 0.05)
        yaw_diff = abs(gt_d['dyaw_deg'] - fl_d['dyaw_deg'])

        report['diagnosis'] = {
            'x_axis_flipped': bool(x_flip),
            'y_axis_flipped': bool(y_flip),
            'yaw_diff_deg': round(yaw_diff, 1),
            'root_cause': '',
        }

        if x_flip and y_flip:
            report['diagnosis']['root_cause'] = 'Both X and Y flipped → extrinsic_R is Rz(180°) (Z-axis rotation), should be Ry(180°)'
        elif x_flip:
            report['diagnosis']['root_cause'] = 'X axis flipped → extrinsic_R X-axis mapping is wrong'
        elif y_flip:
            report['diagnosis']['root_cause'] = 'Y axis flipped → extrinsic_R Y-axis mapping is wrong'
        elif yaw_diff > 90:
            report['diagnosis']['root_cause'] = 'Yaw reversed → extrinsic_R or IMU orientation is inverted'
        else:
            report['diagnosis']['root_cause'] = 'Axes appear aligned (no flip detected)'

        # TF 数据
        if tf_start and tf_end:
            report['tf_start'] = {'x': round(tf_start[1], 3), 'y': round(tf_start[2], 3),
                                   'yaw_deg': round(math.degrees(tf_start[3]), 1)}
            report['tf_end'] = {'x': round(tf_end[1], 3), 'y': round(tf_end[2], 3),
                                 'yaw_deg': round(math.degrees(tf_end[3]), 1)}
            report['tf_delta'] = {
                'dx': round(tf_end[1] - tf_start[1], 3),
                'dy': round(tf_end[2] - tf_start[2], 3),
            }

        return report

    def print_final(self, report):
        if 'error' in report:
            print(f'\n{RED}数据不足: {report["error"]}{NC}')
            return

        print(f'\n{"="*70}')
        print(f'{BOLD}  坐标轴对齐诊断报告{NC}')
        print(f'{"="*70}')
        print(f'  时长: {report["duration_sec"]}s')
        print(f'  样本: GT={report["gt_samples"]}, FL={report["fl_samples"]}, TF={report["tf_samples"]}')
        print(f'{"─"*70}')
        print(f'{BOLD}  全程位移对比{NC}')
        print(f'{"─"*70}')
        print(f'{"源":<16} {"起点 X":>8} {"起点 Y":>8} {"起点 yaw°":>10} {"终点 X":>8} {"终点 Y":>8} {"终点 yaw°":>10}')
        print(f'{"─"*70}')
        print(f'{"GT (真值)":<16} {report["gt_start"]["x"]:>+8.3f} {report["gt_start"]["y"]:>+8.3f} '
              f'{report["gt_start"]["yaw_deg"]:>+10.1f} {report["gt_end"]["x"]:>+8.3f} {report["gt_end"]["y"]:>+8.3f} '
              f'{report["gt_end"]["yaw_deg"]:>+10.1f}')
        print(f'{"FastLIO2":<16} {report["fl_start"]["x"]:>+8.3f} {report["fl_start"]["y"]:>+8.3f} '
              f'{report["fl_start"]["yaw_deg"]:>+10.1f} {report["fl_end"]["x"]:>+8.3f} {report["fl_end"]["y"]:>+8.3f} '
              f'{report["fl_end"]["yaw_deg"]:>+10.1f}')
        if 'tf_end' in report:
            print(f'{"Nav2 TF":<16} {report["tf_start"]["x"]:>+8.3f} {report["tf_start"]["y"]:>+8.3f} '
                  f'{report["tf_start"]["yaw_deg"]:>+10.1f} {report["tf_end"]["x"]:>+8.3f} {report["tf_end"]["y"]:>+8.3f} '
                  f'{report["tf_end"]["yaw_deg"]:>+10.1f}')
        print(f'{"─"*70}')
        print(f'{BOLD}  位移增量{NC}')
        print(f'{"─"*70}')
        gt_d = report['gt_delta']
        fl_d = report['fl_delta']
        print(f'{"GT":<16}  Δx={gt_d["dx"]:+.3f}  Δy={gt_d["dy"]:+.3f}  Δyaw={gt_d["dyaw_deg"]:+.1f}°')
        print(f'{"FastLIO2":<16}  Δx={fl_d["dx"]:+.3f}  Δy={fl_d["dy"]:+.3f}  Δyaw={fl_d["dyaw_deg"]:+.1f}°')
        if 'tf_delta' in report:
            tf_d = report['tf_delta']
            print(f'{"Nav2 TF":<16}  Δx={tf_d["dx"]:+.3f}  Δy={tf_d["dy"]:+.3f}')
        print(f'{"─"*70}')
        print(f'{BOLD}  诊断结论{NC}')
        print(f'{"─"*70}')
        diag = report['diagnosis']
        if diag['x_axis_flipped']:
            print(f'  {RED}⚠ X 轴翻转: GT 和 FastLIO2 的 X 位移方向相反{NC}')
        else:
            print(f'  {GREEN}✓ X 轴: 对齐{NC}')
        if diag['y_axis_flipped']:
            print(f'  {RED}⚠ Y 轴翻转: GT 和 FastLIO2 的 Y 位移方向相反{NC}')
        else:
            print(f'  {GREEN}✓ Y 轴: 对齐{NC}')
        if diag['yaw_diff_deg'] > 90:
            print(f'  {RED}⚠ Yaw 偏差: {diag["yaw_diff_deg"]:.1f}° (>90°){NC}')
        else:
            print(f'  {GREEN}✓ Yaw: 一致{NC}')
        print(f'\n  根因: {diag["root_cause"]}')
        print(f'{"="*70}\n')


def main():
    parser = argparse.ArgumentParser(description='坐标轴对齐诊断')
    parser.add_argument('--auto', action='store_true',
                        help='自动模式: 发 walk_mode + 推 vx=0.2')
    parser.add_argument('--duration', type=int, default=30,
                        help='总时长 (秒, 默认 30)')
    args = parser.parse_args()

    rclpy.init()
    node = AxisChecker(duration=args.duration, auto_mode=args.auto)

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    report = node.compute_final_report()
    node.print_final(report)

    # 保存 JSON
    if 'error' not in report:
        reports_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'reports')
        os.makedirs(reports_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        json_path = os.path.join(reports_dir, f'axis_check_{ts}.json')
        with open(json_path, 'w') as f:
            json.dump(report, f, indent=2)
        print(f'  报告已保存: {json_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
