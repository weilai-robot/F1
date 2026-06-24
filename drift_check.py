#!/usr/bin/env python3
"""
drift_check.py — FastLIO2 SLAM 定位漂移诊断

对比 FastLIO2 /Odometry (camera_init 系) 与 MuJoCo /mujoco/ground_truth (世界系)
在整个导航过程中的位置偏差。

用法:
  python3 drift_check.py 5.0 0.0          # 发目标 (5,0), 默认 90s 超时
  python3 drift_check.py 5.0 0.0 120      # 自定义超时 120s
  python3 drift_check.py --no-goal 60     # 不发导航目标, 只记录 60s 漂移

输出:
  终端实时漂移 + 最终统计报告
  reports/drift_<timestamp>.json
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
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64MultiArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


GREEN  = '\033[0;32m'
YELLOW = '\033[1;33m'
RED    = '\033[0;31m'
CYAN   = '\033[0;36m'
BOLD   = '\033[1m'
NC     = '\033[0m'


class DriftChecker(Node):
    def __init__(self, goal_x, goal_y, timeout_sec, send_goal):
        super().__init__('drift_checker')

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.timeout_sec = timeout_sec
        self.send_goal_flag = send_goal
        self.finished = False
        self.result_status = None
        self.nav_started = False

        # ── 数据缓冲 ──
        # ground_truth: [(sim_t, x, y, z, roll, pitch, yaw)]
        self.gt_data = deque(maxlen=100000)
        # FastLIO2 odometry: [(sim_t, x, y, z, qx, qy, qz, qw)]
        self.fastlio_data = deque(maxlen=100000)
        # 配对数据: [(sim_t, gt_x, gt_y, fl_x, fl_y, drift_xy)]
        self.paired_data = deque(maxlen=100000)

        # ── 最新值缓存 (用于配对) ──
        self._latest_gt = None     # (sim_t, x, y, z, yaw)
        self._latest_fl = None     # (sim_t, x, y, z)

        # ── 实时统计 ──
        self._max_drift = 0.0
        self._drift_samples = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=200,
        )

        # ── 订阅 ──
        self.create_subscription(Float64MultiArray, '/mujoco/ground_truth',
                                  self._gt_cb, sensor_qos)
        self.create_subscription(Odometry, '/Odometry',
                                  self._fastlio_cb, sensor_qos)

        # ── walk_mode + Nav2 ──
        self.walk_pub = self.create_publisher(Float32, '/walk_mode', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        if self.send_goal_flag:
            self._action_client = ActionClient(self, NavigateToPose,
                                               'navigate_to_pose')

        # ── 定时器 ──
        self.create_timer(2.0, self._startup_sequence)
        self.create_timer(1.0, self._timeout_check)
        self.create_timer(5.0, self._report_progress)
        self._start_wall = time.monotonic()
        self._startup_done = False

        self.get_logger().info(
            f'{CYAN}DriftChecker 启动{NC}\n'
            f'  目标: ({goal_x:.1f}, {goal_y:.1f})\n'
            f'  超时: {timeout_sec}s\n'
            f'  发导航目标: {send_goal}'
        )

    # ──────────────── 回调 ────────────────

    def _gt_cb(self, msg: Float64MultiArray):
        """MuJoCo ground truth: [sim_t, x, y, z, roll, pitch, yaw, rtf, collisions, cum_dist]"""
        d = msg.data
        if len(d) < 7:
            return
        sim_t = d[0]
        x, y, z = d[1], d[2], d[3]
        yaw = d[6]
        self._latest_gt = (sim_t, x, y, z, yaw)
        self.gt_data.append((sim_t, x, y, yaw))

        self._try_pair()

    def _fastlio_cb(self, msg: Odometry):
        """FastLIO2 /Odometry: camera_init frame"""
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._latest_fl = (sim_t, p.x, p.y, p.z)
        self.fastlio_data.append((sim_t, p.x, p.y, p.z))

        self._try_pair()

    def _try_pair(self):
        """时间最近邻配对 + 漂移计算"""
        if self._latest_gt is None or self._latest_fl is None:
            return

        gt_t, gt_x, gt_y, gt_z, gt_yaw = self._latest_gt
        fl_t, fl_x, fl_y, fl_z = self._latest_fl

        # 只在时间差 < 0.1s 时配对
        if abs(gt_t - fl_t) > 0.1:
            return

        # FastLIO2 在 camera_init 系 (雷达初始位置)
        # camera_init → map 有 Z=1.31 的偏移 + 可能的旋转
        # odom_bridge 把 FastLIO2 的 (x,y) 直接映射到 odom→base_footprint
        # 所以直接比较 (gt_x, gt_y) vs (fl_x, fl_y) 是合理的
        # (因为 camera_init 在 (0,0,0) 初始化, map 在 (0,0,0), 无额外旋转)
        drift_x = fl_x - gt_x
        drift_y = fl_y - gt_y
        drift_z = fl_z - gt_z
        drift_xy = math.sqrt(drift_x**2 + drift_y**2)

        self.paired_data.append((gt_t, gt_x, gt_y, gt_yaw,
                                  fl_x, fl_y, drift_x, drift_y, drift_xy))

        self._max_drift = max(self._max_drift, drift_xy)
        self._drift_samples += 1

    def _cmd_vel_cb(self, msg: Twist):
        """检测 Nav2 是否开始发速度 → 导航已开始"""
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            if not self.nav_started:
                self.nav_started = True
                self.get_logger().info(f'{GREEN}导航已开始 (检测到 cmd_vel){NC}')

    # ──────────────── 定时器 ────────────────

    def _startup_sequence(self):
        """启动序列: walk_mode → nav goal"""
        if self._startup_done:
            return

        # 1. 发 walk_mode
        self.get_logger().info(f'{YELLOW}[1/2] 发送 walk_mode...{NC}')
        for _ in range(15):  # 5Hz × 3s
            msg = Float32()
            msg.data = 0.0
            self.walk_pub.publish(msg)
            time.sleep(0.2)

        self.get_logger().info(f'{YELLOW}      walk_mode 已发送, 等待 3s...{NC}')
        time.sleep(3)

        # 2. 发导航目标
        if self.send_goal_flag:
            if not self._action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error('Nav2 action server 不可用')
                self.result_status = 'NO_NAV2'
                self.finished = True
                self._startup_done = True
                return

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = self.goal_x
            goal_msg.pose.pose.position.y = self.goal_y
            goal_msg.pose.pose.orientation.w = 1.0

            self.get_logger().info(
                f'{YELLOW}[2/2] 发送导航目标: ({self.goal_x:.1f}, {self.goal_y:.1f}){NC}')

            future = self._action_client.send_goal_async(goal_msg)
            future.add_done_callback(self._goal_response_cb)
        else:
            self.get_logger().info(f'{YELLOW}[2/2] 跳过导航目标, 仅记录漂移{NC}')

        self._startup_done = True

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 拒绝目标')
            self.result_status = 'REJECTED'
            self.finished = True
            return
        self.get_logger().info(f'{GREEN}Nav2 接受目标, 导航中...{NC}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.result_status = 'SUCCEEDED'
        else:
            self.result_status = f'FAILED_{status}'
        self.finished = True

    def _timeout_check(self):
        if self.finished:
            return
        elapsed = time.monotonic() - self._start_wall
        if elapsed >= self.timeout_sec:
            self.get_logger().warn(f'超时 ({self.timeout_sec}s)')
            self.result_status = 'TIMEOUT'
            self.finished = True

    def _report_progress(self):
        if self._drift_samples > 0:
            latest = self.paired_data[-1]
            elapsed = time.monotonic() - self._start_wall
            self.get_logger().info(
                f'{CYAN}[{elapsed:.0f}s] GT=({latest[1]:.2f},{latest[2]:.2f}) '
                f'FL=({latest[4]:.2f},{latest[5]:.2f}) '
                f'drift={latest[8]:.3f}m max={self._max_drift:.3f}m{NC}'
            )

    # ──────────────── 最终报告 ────────────────

    def compute_report(self):
        if len(self.paired_data) < 2:
            print(f'\n{RED}样本不足 ({len(self.paired_data)} 帧), 无法统计{NC}')
            return None

        drifts_xy = [d[8] for d in self.paired_data]
        drifts_x  = [d[6] for d in self.paired_data]
        drifts_y  = [d[7] for d in self.paired_data]

        n = len(drifts_xy)
        mean_d = sum(drifts_xy) / n
        max_d  = max(drifts_xy)
        min_d  = min(drifts_xy)
        rms_d  = math.sqrt(sum(d**2 for d in drifts_xy) / n)

        mean_dx = sum(drifts_x) / n
        mean_dy = sum(drifts_y) / n

        # p95
        sorted_d = sorted(drifts_xy)
        p95_idx = int(n * 0.95)
        p95_d = sorted_d[min(p95_idx, n-1)]

        # 漂移率: 最大漂移 / 行走距离
        total_dist = 0.0
        for i in range(1, len(self.paired_data)):
            dx = self.paired_data[i][1] - self.paired_data[i-1][1]
            dy = self.paired_data[i][2] - self.paired_data[i-1][2]
            total_dist += math.sqrt(dx*dx + dy*dy)

        drift_rate = (max_d / total_dist * 100) if total_dist > 0.01 else 0

        report = {
            'samples': n,
            'ground_truth_frames': len(self.gt_data),
            'fastlio_frames': len(self.fastlio_data),
            'total_distance_m': round(total_dist, 2),
            'drift_mean_m': round(mean_d, 4),
            'drift_rms_m': round(rms_d, 4),
            'drift_max_m': round(max_d, 4),
            'drift_min_m': round(min_d, 4),
            'drift_p95_m': round(p95_d, 4),
            'drift_mean_x_m': round(mean_dx, 4),
            'drift_mean_y_m': round(mean_dy, 4),
            'drift_rate_pct': round(drift_rate, 1),
            'nav_result': self.result_status,
        }
        return report

    def print_report(self, report):
        if report is None:
            return

        print(f'\n{"="*60}')
        print(f'{BOLD}  FastLIO2 SLAM 定位漂移报告{NC}')
        print(f'{"="*60}')
        print(f'  导航结果:     {report["nav_result"]}')
        print(f'  配对样本数:   {report["samples"]}')
        print(f'  GT 帧数:      {report["ground_truth_frames"]}')
        print(f'  FastLIO2 帧数:{report["fastlio_frames"]}')
        print(f'  行走距离:     {report["total_distance_m"]:.2f} m')
        print(f'{"─"*60}')
        print(f'{BOLD}  漂移统计 (XY 平面){NC}')
        print(f'{"─"*60}')

        # 判定等级
        if report['drift_mean_m'] < 0.1:
            grade = f'{GREEN}✓ 优秀{NC}'
        elif report['drift_mean_m'] < 0.3:
            grade = f'{YELLOW}⚠ 可接受{NC}'
        elif report['drift_mean_m'] < 0.5:
            grade = f'{YELLOW}⚠ 较差{NC}'
        else:
            grade = f'{RED}✗ 严重漂移{NC}'

        print(f'  平均漂移:     {report["drift_mean_m"]:.4f} m    {grade}')
        print(f'  RMS 漂移:     {report["drift_rms_m"]:.4f} m')
        print(f'  最大漂移:     {report["drift_max_m"]:.4f} m')
        print(f'  最小漂移:     {report["drift_min_m"]:.4f} m')
        print(f'  P95 漂移:     {report["drift_p95_m"]:.4f} m')
        print(f'{"─"*60}')
        print(f'{BOLD}  方向分解{NC}')
        print(f'{"─"*60}')
        print(f'  X 方向平均:   {report["drift_mean_x_m"]:+.4f} m  '
              f'({"东偏" if report["drift_mean_x_m"] > 0 else "西偏"})')
        print(f'  Y 方向平均:   {report["drift_mean_y_m"]:+.4f} m  '
              f'({"北偏" if report["drift_mean_y_m"] > 0 else "南偏"})')
        print(f'{"─"*60}')
        print(f'  漂移率:       {report["drift_rate_pct"]:.1f}% '
              f'(最大漂移/行走距离)')
        print(f'{"="*60}')

        if report['drift_mean_m'] > 0.3:
            print(f'\n{RED}⚠ 建议:{NC}')
            print(f'  漂移 > 0.3m 会严重影响 Nav2 导航:')
            print(f'  - 机器人在 costmap 中位置偏差 → 路径不对齐')
            print(f'  - 点云偏移 → 障碍物位置不准 → 撞墙')
            print(f'  方案: 仿真中使用 MuJoCo ground truth 替代 FastLIO2 定位')
        elif report['drift_mean_m'] > 0.1:
            print(f'\n{YELLOW}⚠ 注意:{NC} 漂移在可接受范围但偏高')
            print(f'  点云在 RViz 中可能有轻微偏移')

        print()


def main():
    parser = argparse.ArgumentParser(
        description='FastLIO2 SLAM 定位漂移诊断')
    parser.add_argument('goal_x', type=float, nargs='?', default=0.0,
                        help='目标 X (默认 0)')
    parser.add_argument('goal_y', type=float, nargs='?', default=0.0,
                        help='目标 Y (默认 0)')
    parser.add_argument('timeout', type=int, nargs='?', default=90,
                        help='超时秒数 (默认 90)')
    parser.add_argument('--no-goal', action='store_true',
                        help='不发导航目标, 只记录漂移')
    args = parser.parse_args()

    rclpy.init()
    node = DriftChecker(
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        timeout_sec=args.timeout if not args.no_goal else int(args.goal_x) if args.goal_x else 60,
        send_goal=not args.no_goal,
    )

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.result_status = 'INTERRUPTED'
        node.finished = True

    report = node.compute_report()
    node.print_report(report)

    # 保存 JSON
    if report:
        reports_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'reports')
        os.makedirs(reports_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        json_path = os.path.join(reports_dir, f'drift_{ts}.json')
        with open(json_path, 'w') as f:
            json.dump(report, f, indent=2)
        print(f'  报告已保存: {json_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
