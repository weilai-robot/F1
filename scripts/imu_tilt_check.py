#!/usr/bin/env python3
"""
imu_tilt_check.py — /livox/imu 倾角读数（吊架调平 / mount 残差三时点对账）

被动订阅 IMU，滚动平均 5~10s，打印：朝向自检（fz 符号）+ 总倾角 θ + δx/δy + 调平方向指引。
只订阅、不发布、不碰任何链路。规格与判读阈值：docs/sim2real/真机建图SOP-机器人吊架native-2026-10-26.md §6。

用法（先 source /opt/ros/humble/setup.bash）：
  python3 imu_tilt_check.py                       # /livox/imu，8s 窗，1Hz 刷新
  python3 imu_tilt_check.py --window 10 --rate 0.5
  旧包回放验证（小车正装 → fz 应为正、静止段小倾角；按 header.stamp 窗口化，无需 --clock）：
  ros2 bag play ~/humanoid_ws/rosbag/fastlio_test

物理口径（2026-10-26 旧包回放钉死）：
  静止时加速度计读数 = 比力 = 世界上方向；livox 驱动原样发 **g 单位**（非 m/s²，实测 |f|≈0.99），
  倾角计算只用比值、与单位无关。正装 fz>0（z 轴朝物理上），倒装 fz<0（FAST-LIO mean_acc=(0,0,-1) 同口径）。
  (fx,fy) 是世界上方向在传感器水平面的投影，指向传感器【偏高】的一侧 —— 压低偏高侧即调平。
  首次上吊架先轻推一侧对账方向（"+x 是机器人哪边"），10 秒钉死，勿凭记忆。
判读：
  吊架调平达标 <0.3°（只动吊带/挂点）；导航期残差达标 <0.1°（地面站立基准）。
  吊架上勿动 mount 垫片调零——会把吊挂姿态 β 焊进 mount 残差 δ（SOP §6.2）。
"""

import argparse
import collections
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

G_MIN, G_MAX = 0.90, 1.10   # 静止重力幅值合理带（g 单位，livox 驱动不转 m/s²）
MOVE_GYRO = 0.03           # 平均角速度超过此值判"在动"，倾角不可信（rad/s）
LEVEL_RIG, LEVEL_RES = 0.3, 0.1  # 吊架 / 残差达标阈值（度）


class ImuTiltCheck(Node):

    def __init__(self, topic: str, window_s: float, rate_hz: float):
        super().__init__('imu_tilt_check')
        self.buf = collections.deque()
        self.window_s = float(window_s)
        qos = QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, topic, self._cb, qos)
        self.create_timer(1.0 / rate_hz, self._report)
        self.get_logger().info(
            f'被动订阅 {topic}（滚动窗口 {window_s:.0f}s）——静置后读数，Ctrl+C 退出')

    def _cb(self, m: Imu):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t <= 0.0:  # stamp 缺失兜底：退节点时钟
            t = self.get_clock().now().nanoseconds * 1e-9
        a, w = m.linear_acceleration, m.angular_velocity
        self.buf.append((t, (a.x, a.y, a.z), math.hypot(w.x, w.y, w.z)))
        while self.buf and self.buf[0][0] < t - self.window_s:
            self.buf.popleft()
        if len(self.buf) > int(self.window_s * 300):  # stamp 异常时的内存兜底
            self.buf.popleft()

    def _report(self):
        if len(self.buf) < 100:  # 至少 ~0.5s 数据才出读数
            return
        n = len(self.buf)
        fx = sum(b[1][0] for b in self.buf) / n
        fy = sum(b[1][1] for b in self.buf) / n
        fz = sum(b[1][2] for b in self.buf) / n
        gyro = sum(b[2] for b in self.buf) / n
        span = self.buf[-1][0] - self.buf[0][0]
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        az = abs(fz)
        tilt = math.degrees(math.atan2(math.hypot(fx, fy), az))
        dx = math.degrees(math.atan2(fx, az))   # 正 = 传感器 +x 侧偏高
        dy = math.degrees(math.atan2(fy, az))   # 正 = 传感器 +y 侧偏高

        if fz < 0:
            orient = '倒装（fz<0，z 轴朝物理下）—— 与 FAST-LIO mean_acc=(0,0,-1) 口径一致'
        elif fz > 0:
            orient = '正装（fz>0，z 轴朝物理上）'
        else:
            orient = '⚠ fz≈0：近水平侧放？安装姿态异常'
        moving = f'⚠ 在动（gyro={gyro:.3f} rad/s），倾角读数不可信' if gyro > MOVE_GYRO else '静止 ✓'
        if tilt < LEVEL_RES:
            verdict = f'✓✓ 残差级（<{LEVEL_RES}°，导航期达标）'
        elif tilt < LEVEL_RIG:
            verdict = f'✓ 吊架达标（<{LEVEL_RIG}°）'
        else:
            verdict = f'✗ 未达标（≥{LEVEL_RIG}°）'
        unit = '' if G_MIN <= mag <= G_MAX else \
            f'  ⚠ |f|={mag:.2f} 异常（期望 ~1.0 g，查传感器）'

        hi = []
        if abs(dx) >= 0.1:
            hi.append(f"传感器 {'+' if dx > 0 else '-'}x 侧（δx={dx:+.2f}°）")
        if abs(dy) >= 0.1:
            hi.append(f"传感器 {'+' if dy > 0 else '-'}y 侧（δy={dy:+.2f}°）")
        guide = ('压低 ' + '、'.join(hi) +
                 ' —— 动吊带/挂点，勿动 mount 垫片（β/δ 混淆，SOP §6.2）') if hi \
            else '两侧均 <0.1°，无需调整'

        print(f'[样本 {n} · {span:.1f}s] |f|={mag:.2f} g{unit} · {moving}\n'
              f'  朝向: {orient}\n'
              f'  总倾角 θ = {tilt:.2f}°  {verdict}\n'
              f'    δx={dx:+.2f}°（正=+x 侧偏高）  δy={dy:+.2f}°（正=+y 侧偏高）\n'
              f'  调平: {guide}\n')


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--topic', default='/livox/imu')
    p.add_argument('--window', type=float, default=8.0, help='滚动平均窗口秒数（默认 8）')
    p.add_argument('--rate', type=float, default=1.0, help='打印频率 Hz（默认 1）')
    a = p.parse_args()
    rclpy.init()
    node = ImuTiltCheck(a.topic, a.window, a.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C / SIGTERM 均干净退出
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
