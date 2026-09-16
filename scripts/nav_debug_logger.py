#!/usr/bin/env python3
"""
nav_debug_logger.py — 导航调试日志中继节点（真机）

把 MPPI 的 /trajectories (visualization_msgs/MarkerArray, 单帧含 ~1300 个
候选轨迹 marker) 中 "Optimal Trajectory" 命名空间的轨迹点提取出来,
转成紧凑的 nav_msgs/Path 发布:

    /trajectories ──(本节点)──> /nav_debug/mppi_optimal_traj   (frame=odom, 10 Hz)

为什么要中继:
  1. /trajectories 单帧 ~300KB (候选轨迹占绝大部分), 直接录包 ~3MB/s;
     最优轨迹单帧仅 ~3KB, 长时间录包无压力。
  2. MPPI 仅在 /trajectories 有订阅者时才实际发布
     (visualize=true 且 subscription_count > 0)。
     本节点起"保活订阅"作用, 保证录包期间数据不断流。

坐标: /trajectories 的 marker 在 local costmap 全局系 (= odom) 下;
      本节点原样透传, 统一到 map 系由 nav_debug_plot.py 用 map->odom TF 完成。

前提:
  navigation 侧 nav2_real.yaml 中 FollowPath.visualize = true (已默认启用)。
  若 /trajectories 一直无数据, 检查:
    ros2 param get /controller_server FollowPath.visualize

用法:
  python3 nav_debug_logger.py        # 常驻, Ctrl+C 退出（退出时打印统计）

注意: 由 nav_debug_record.sh 自动拉起, 一般无需手动运行。
"""
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

# MPPI TrajectoryVisualizer 中最优轨迹的命名空间 (nav2_mppi_controller/controller.cpp)
OPTIMAL_NS = "Optimal Trajectory"

INPUT_TOPIC = "/trajectories"
OUTPUT_TOPIC = "/nav_debug/mppi_optimal_traj"


class NavDebugLogger(Node):
    def __init__(self):
        super().__init__("nav_debug_logger")

        self._pub = self.create_publisher(Path, OUTPUT_TOPIC, 10)
        self._sub = self.create_subscription(
            MarkerArray, INPUT_TOPIC, self._on_markers, 10
        )

        self._n_rx = 0            # 收到的 /trajectories 帧数
        self._n_relay = 0         # 成功中继的最优轨迹数
        self._last_rx_t = None    # 最近一次收到消息的时刻
        self._t_start = time.monotonic()
        self._warned_silent = False
        self._warned_empty = False

        # 10s 巡检: 静默/空帧给出一次性告警, 避免刷屏
        self._timer = self.create_timer(10.0, self._check_health)

        self.get_logger().info(
            f"启动: {INPUT_TOPIC} -> {OUTPUT_TOPIC} "
            f"(提取命名空间 '{OPTIMAL_NS}')"
        )

    def _check_health(self):
        elapsed = time.monotonic() - self._t_start
        if self._n_rx == 0 and not self._warned_silent and elapsed > 12.0:
            self._warned_silent = True
            self.get_logger().warn(
                f"{elapsed:.0f}s 未收到 {INPUT_TOPIC}。请检查: "
                "1) Nav2 controller_server 已激活 "
                "2) FollowPath.visualize=true "
                "(ros2 param get /controller_server FollowPath.visualize)"
            )
        elif self._n_rx > 0 and self._n_relay == 0 and not self._warned_empty:
            self._warned_empty = True
            self.get_logger().warn(
                f"收到 {INPUT_TOPIC} 但无 '{OPTIMAL_NS}' markers "
                "(RotationShim 原地旋转阶段 MPPI 不更新, 属正常短暂现象)"
            )

    def _on_markers(self, msg: MarkerArray):
        self._n_rx += 1
        self._last_rx_t = time.monotonic()

        pts = [m for m in msg.markers if m.ns == OPTIMAL_NS]
        if not pts:
            return
        pts.sort(key=lambda m: m.id)  # marker id 递增 = 轨迹点先后顺序

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        frame = pts[0].header.frame_id
        # marker header.stamp 为 0 (MPPI 未填充), frame_id 正常为 "odom"
        path.header.frame_id = frame if frame else "odom"

        path.poses = []
        for m in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose = m.pose
            path.poses.append(ps)

        self._pub.publish(path)
        self._n_relay += 1

    def print_stats(self):
        self.get_logger().info(
            f"统计: {INPUT_TOPIC} 收到 {self._n_rx} 帧, "
            f"中继最优轨迹 {self._n_relay} 帧, 输出 {OUTPUT_TOPIC}"
        )


def main():
    rclpy.init(args=None)
    node = NavDebugLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_stats()
        node.destroy_node()
        # 被外部非优雅杀掉 (kill -9) 时 ros2 可能已关闭, 容错处理
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
