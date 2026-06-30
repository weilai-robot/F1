#!/usr/bin/env python3
"""
nav_test_runner.py — 导航仿真自动化测试 + 指标计算

用法:
  # 前提: run_sim_nav.sh 已启动, 机器人已切到 walk_mode
  python3 nav_test_runner.py --goal-x 5.0 --goal-y 0.0 --timeout 60

  # 批量跑多个场景
  python3 nav_test_runner.py --batch

输出:
  reports/<scenario>_<timestamp>.json   — 结构化结果
  reports/<scenario>_<timestamp>.md     — 可读报告
  reports/latest.json                   — 最新结果软链

指标:
  P0: 摔倒率, 碰撞次数
  P1: 导航成功率, 位置精度, SLAM漂移
  P2: 速度jerk, 路径效率, 完成时间
  性能: RTF, CPU/内存 (可选 pidstat)
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ═══════════════════════════════════════════════════════════
#  指标计算
# ═══════════════════════════════════════════════════════════

class MetricsCalculator:
    """收集原始数据 + 计算所有指标"""

    # 摔倒检测阈值
    FALL_Z_THRESHOLD = 0.35      # base_z < 0.35m → 摔倒
    FALL_ANGLE_THRESHOLD = math.radians(45)  # pitch/roll > 45° → 摔倒

    def __init__(self):
        # ground truth 数据
        self.gt_data = []        # [(sim_time, x, y, z, roll, pitch, yaw, rtf, collisions, cum_dist)]
        # cmd_vel 数据
        self.cmd_vel_data = []   # [(wall_time, vx, vy, wz)]
        # SLAM odom 数据
        self.odom_data = []      # [(sim_time, x, y)]
        # odom-GT 逐帧配对数据 (用于漂移统计)
        self.odom_paired_data = []  # [(sim_t, gt_x, gt_y, odom_x, odom_y, drift_xy)]

        self.start_time = None     # goal 发出时刻 (wall clock)
        self.goal_sent_time = None  # goal 发出时刻 (wall clock, 用于 plan_time)
        self.first_motion_time = None  # 首次检测到非零 cmd_vel 的时刻
        self.end_time = None
        self.fall_detected = False
        self.collision_total = 0
        self.max_rtf = 0.0
        self.min_rtf = 1e9

    def add_ground_truth(self, data: list):
        """data = [sim_t, x, y, z, roll, pitch, yaw, rtf, collisions, cum_dist]"""
        self.gt_data.append(tuple(data))

        # 实时检测摔倒
        z = data[3]
        roll = abs(data[4])
        pitch = abs(data[5])
        if z < self.FALL_Z_THRESHOLD or roll > self.FALL_ANGLE_THRESHOLD or pitch > self.FALL_ANGLE_THRESHOLD:
            self.fall_detected = True

        # 累计碰撞
        self.collision_total = max(self.collision_total, data[8])

        # RTF 统计
        rtf = data[7]
        if rtf > 0:
            self.max_rtf = max(self.max_rtf, rtf)
            self.min_rtf = min(self.min_rtf, rtf)

    def add_cmd_vel(self, wall_time: float, vx: float, vy: float, wz: float):
        self.cmd_vel_data.append((wall_time, vx, vy, wz))

    def check_first_motion(self, wall_time: float, vx: float, wz: float):
        """检测 goal 发出后首次有速度输出 (用于计算 plan_time)"""
        if self.goal_sent_time and self.first_motion_time is None:
            if abs(vx) > 0.01 or abs(wz) > 0.01:
                self.first_motion_time = wall_time

    def add_odom(self, sim_time: float, x: float, y: float):
        self.odom_data.append((sim_time, x, y))
        # 逐帧配对 GT vs odom (时间最近邻, 窗口 0.1s)
        if self.gt_data:
            best_gt = min(self.gt_data, key=lambda g: abs(g[0] - sim_time))
            if abs(best_gt[0] - sim_time) < 0.1:
                drift = math.sqrt((x - best_gt[1])**2 + (y - best_gt[2])**2)
                self.odom_paired_data.append(
                    (sim_time, best_gt[1], best_gt[2], x, y, drift))

    def compute_metrics(self, goal_x: float, goal_y: float, timeout_sec: float) -> dict:
        """计算全部指标"""
        m = {}

        # === P0: 摔倒 + 碰撞 ===
        m["fall"] = self.fall_detected
        m["collisions"] = int(self.collision_total)

        # === P1: 成功/精度/漂移 ===
        if len(self.gt_data) == 0:
            m["success"] = False
            m["error_reason"] = "no_ground_truth_data"
            return m

        final_gt = self.gt_data[-1]
        final_x, final_y = final_gt[1], final_gt[2]
        goal_dist = math.sqrt((final_x - goal_x)**2 + (final_y - goal_y)**2)

        m["position_error_m"] = round(goal_dist, 3)
        m["success"] = (not self.fall_detected) and (m["collisions"] == 0) and (goal_dist < 0.35)

        # === 规划时间: goal发出 → 首次有速度输出 ===
        if self.goal_sent_time and self.first_motion_time:
            m["plan_time_s"] = round(self.first_motion_time - self.goal_sent_time, 2)
        else:
            m["plan_time_s"] = None

        # === 定位精度: 逐帧 GT vs odom 漂移统计 ===
        if len(self.odom_paired_data) > 0:
            import numpy as np
            drifts = np.array([d[5] for d in self.odom_paired_data])
            m["drift_mean_m"] = round(float(drifts.mean()), 4)
            m["drift_rms_m"] = round(float(np.sqrt((drifts**2).mean())), 4)
            m["drift_max_m"] = round(float(drifts.max()), 4)
            sorted_d = np.sort(drifts)
            m["drift_p95_m"] = round(float(sorted_d[int(len(sorted_d)*0.95)]), 4)
        else:
            m["drift_mean_m"] = None
            m["drift_rms_m"] = None
            m["drift_max_m"] = None
            m["drift_p95_m"] = None

        # 兼容旧字段名 (仅末帧)
        m["slam_drift_m"] = m["drift_max_m"]

        # === P2: 完成时间/路径效率/jerk ===
        if self.start_time and self.end_time:
            sim_duration = self.gt_data[-1][0] - self.gt_data[0][0] if len(self.gt_data) > 1 else 0
            m["completion_time_s"] = round(sim_duration, 2)
        else:
            m["completion_time_s"] = None

        # 路径效率: 直线距离 / 累计位移
        if len(self.gt_data) >= 2:
            start_x, start_y = self.gt_data[0][1], self.gt_data[0][2]
            straight_dist = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
            cum_dist = final_gt[9]
            m["path_efficiency"] = round(straight_dist / max(cum_dist, 0.01), 3) if cum_dist > 0.01 else None
        else:
            m["path_efficiency"] = None

        # 速度 jerk
        jerk_stats = self._compute_jerk()
        m["linear_jerk_rms"] = jerk_stats["linear_rms"]
        m["angular_jerk_rms"] = jerk_stats["angular_rms"]
        m["direction_reversals_per_sec"] = jerk_stats["reversal_rate"]

        # === 速度统计: 从 GT 轨迹差分 ===
        v_stats = self._compute_velocity_stats()
        m.update(v_stats)

        # === 转弯半径: 从 GT 轨迹曲率 ===
        m["turning_radius_min_m"] = self._compute_min_turning_radius()

        # === 性能 ===
        if self.gt_data:
            rtfs = [g[7] for g in self.gt_data if g[7] > 0]
            if rtfs:
                m["rtf_mean"] = round(sum(rtfs) / len(rtfs), 3)
                m["rtf_min"] = round(min(rtfs), 3)
            else:
                m["rtf_mean"] = None
                m["rtf_min"] = None
        else:
            m["rtf_mean"] = None
            m["rtf_min"] = None

        m["timeout_s"] = timeout_sec
        m["timed_out"] = (m.get("completion_time_s") or 0) >= timeout_sec * 0.95

        return m

    def _compute_velocity_stats(self) -> dict:
        """从 GT 轨迹逐帧差分计算实际线速度统计"""
        if len(self.gt_data) < 3:
            return {"vmax_m_s": None, "vmin_m_s": None,
                    "vmean_m_s": None, "vstd_m_s": None}

        velocities = []
        for i in range(1, len(self.gt_data)):
            dt = self.gt_data[i][0] - self.gt_data[i-1][0]
            if dt < 1e-6:
                continue
            dx = self.gt_data[i][1] - self.gt_data[i-1][1]
            dy = self.gt_data[i][2] - self.gt_data[i-1][2]
            v = math.sqrt(dx*dx + dy*dy) / dt
            velocities.append(v)

        if not velocities:
            return {"vmax_m_s": None, "vmin_m_s": None,
                    "vmean_m_s": None, "vstd_m_s": None}

        import numpy as np
        v_arr = np.array(velocities)
        return {
            "vmax_m_s": round(float(v_arr.max()), 3),
            "vmin_m_s": round(float(v_arr.min()), 3),
            "vmean_m_s": round(float(v_arr.mean()), 3),
            "vstd_m_s": round(float(v_arr.std()), 3),
        }

    def _compute_min_turning_radius(self):
        """从 GT 轨迹三点法计算最小转弯半径 (外接圆法)"""
        if len(self.gt_data) < 3:
            return None

        min_radius = float('inf')
        for i in range(1, len(self.gt_data) - 1):
            x1, y1 = self.gt_data[i-1][1], self.gt_data[i-1][2]
            x2, y2 = self.gt_data[i][1],   self.gt_data[i][2]
            x3, y3 = self.gt_data[i+1][1], self.gt_data[i+1][2]

            # 三角形边长
            a = math.sqrt((x2-x3)**2 + (y2-y3)**2)
            b = math.sqrt((x1-x3)**2 + (y1-y3)**2)
            c = math.sqrt((x1-x2)**2 + (y1-y2)**2)

            # 半周长
            s = (a + b + c) / 2

            # 三角形面积 (海伦公式)
            area_sq = s * (s-a) * (s-b) * (s-c)
            if area_sq < 1e-8:  # 近似共线, 跳过
                continue

            area = math.sqrt(area_sq)

            # 外接圆半径: R = abc / (4 * Area)
            radius = (a * b * c) / (4 * area)

            if radius < min_radius:
                min_radius = radius

        return round(min_radius, 2) if min_radius != float('inf') else None

    def _compute_jerk(self) -> dict:
        """从 cmd_vel 时间序列计算 jerk"""
        if len(self.cmd_vel_data) < 3:
            return {"linear_rms": None, "angular_rms": None, "reversal_rate": None}

        times = [d[0] for d in self.cmd_vel_data]
        vxs = [d[1] for d in self.cmd_vel_data]
        wzs = [d[3] for d in self.cmd_vel_data]

        # jerk = d(acceleration)/dt ≈ Δ(Δv/Δt)/Δt
        linear_jerks = []
        angular_jerks = []
        for i in range(2, len(times)):
            dt1 = times[i-1] - times[i-2]
            dt2 = times[i] - times[i-1]
            if dt1 < 1e-6 or dt2 < 1e-6:
                continue
            a1 = (vxs[i-1] - vxs[i-2]) / dt1
            a2 = (vxs[i] - vxs[i-1]) / dt2
            dt_mid = (times[i] - times[i-2]) / 2
            if dt_mid > 1e-6:
                linear_jerks.append((a2 - a1) / dt_mid)

            w1 = (wzs[i-1] - wzs[i-2]) / dt1
            w2 = (wzs[i] - wzs[i-1]) / dt2
            if dt_mid > 1e-6:
                angular_jerks.append((w2 - w1) / dt_mid)

        # 方向反转次数
        reversals = 0
        for i in range(1, len(wzs)):
            if wzs[i] * wzs[i-1] < 0 and abs(wzs[i]) > 0.05:
                reversals += 1

        duration = times[-1] - times[0] if len(times) > 1 else 1

        import numpy as np
        lin_rms = float(np.sqrt(np.mean(np.square(linear_jerks)))) if linear_jerks else 0.0
        ang_rms = float(np.sqrt(np.mean(np.square(angular_jerks)))) if angular_jerks else 0.0

        return {
            "linear_rms": round(lin_rms, 3),
            "angular_rms": round(ang_rms, 3),
            "reversal_rate": round(reversals / max(duration, 0.1), 2),
        }


# ═══════════════════════════════════════════════════════════
#  测试执行节点
# ═══════════════════════════════════════════════════════════

class NavTestNode(Node):
    """导航测试执行器: 发 goal + 收数据 + 超时控制"""

    def __init__(self, goal_x, goal_y, goal_yaw, timeout_sec):
        super().__init__("nav_test_runner")

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
        self.timeout_sec = timeout_sec
        self.metrics = MetricsCalculator()
        self.finished = False
        self.result_status = None
        self._cmd_vel_received = False

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=200,
        )

        # 订阅
        self.create_subscription(Float64MultiArray, "/mujoco/ground_truth",
                                  self._gt_cb, sensor_qos)
        self.create_subscription(Twist, "/cmd_vel_limiter",
                                  self._cmd_vel_cb, 10)
        self.create_subscription(Odometry, "/Odometry",
                                  self._odom_cb, sensor_qos)

        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # 超时定时器
        self.create_timer(1.0, self._timeout_check)
        self._start_wall = time.monotonic()

        self.get_logger().info(
            f"NavTest 启动: goal=({goal_x:.1f}, {goal_y:.1f}, yaw={math.degrees(goal_yaw):.0f}°)  timeout={timeout_sec}s"
        )

    def _gt_cb(self, msg: Float64MultiArray):
        self.metrics.add_ground_truth(msg.data)

    def _cmd_vel_cb(self, msg: Twist):
        wall_t = time.monotonic()
        self.metrics.add_cmd_vel(wall_t, msg.linear.x, msg.linear.y, msg.angular.z)
        self.metrics.check_first_motion(wall_t, msg.linear.x, msg.angular.z)

    def _odom_cb(self, msg: Odometry):
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.metrics.add_odom(sim_t, msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _timeout_check(self):
        if self.finished:
            return
        elapsed = time.monotonic() - self._start_wall
        if elapsed >= self.timeout_sec:
            self.get_logger().warn(f"超时 ({self.timeout_sec}s)，终止测试")
            self.result_status = "TIMEOUT"
            self.finished = True

    def send_goal(self):
        """发送 NavigateToPose goal"""
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server 不可用 (10s)")
            self.result_status = "NO_NAV2"
            self.finished = True
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0

        # yaw → quaternion
        cy = math.cos(self.goal_yaw * 0.5)
        sy = math.sin(self.goal_yaw * 0.5)
        goal_msg.pose.pose.orientation.w = cy
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = sy

        self.get_logger().info(f"发送导航目标: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        self.metrics.start_time = time.monotonic()
        self.metrics.goal_sent_time = self.metrics.start_time

        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        pass  # 可加日志，但避免刷屏

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝了目标")
            self.result_status = "REJECTED"
            self.finished = True
            return
        self.get_logger().info("Nav2 接受目标，导航中...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✓ Nav2 报告导航成功")
            self.result_status = "SUCCEEDED"
        else:
            self.get_logger().warn(f"Nav2 报告失败 (status={status})")
            self.result_status = f"FAILED_{status}"
        self.metrics.end_time = time.monotonic()
        self.finished = True


# ═══════════════════════════════════════════════════════════
#  场景定义
# ═══════════════════════════════════════════════════════════

SCENARIOS = {
    "A_straight_5m": {
        "desc": "直线通行 5m (基线)",
        "goal_x": 5.0, "goal_y": 0.0, "goal_yaw": 0.0,
        "timeout": 60,
    },
    "B_obstacle_bypass": {
        "desc": "绕障碍物到 5m 处",
        "goal_x": 5.0, "goal_y": 0.0, "goal_yaw": 0.0,
        "timeout": 60,
    },
    "C_narrow_passage": {
        "desc": "穿越狭窄通道A (0.8m) 到东侧",
        "goal_x": 5.0, "goal_y": -3.0, "goal_yaw": 0.0,
        "timeout": 90,
    },
    "D_impassable": {
        "desc": "不可通过通道B (应绕路)",
        "goal_x": 5.0, "goal_y": 3.2, "goal_yaw": 0.0,
        "timeout": 90,
    },
    "E_long_distance": {
        "desc": "长距离导航 (对角 ~12m)",
        "goal_x": 8.0, "goal_y": -3.0, "goal_yaw": 0.0,
        "timeout": 120,
    },
    "F_return_trip": {
        "desc": "往返导航 (去5m再回原点)",
        "goal_x": 0.0, "goal_y": 0.0, "goal_yaw": 3.14159,
        "timeout": 120,
    },
}


# ═══════════════════════════════════════════════════════════
#  报告生成
# ═══════════════════════════════════════════════════════════

PASS = "✅"
FAIL = "❌"
WARN = "⚠️"


def format_report(scenario_name: str, scenario_desc: str, params: dict,
                  metrics: dict, timestamp: str) -> str:
    """生成 Markdown 报告"""

    status_icon = PASS if metrics.get("success") else FAIL
    fall_icon = FAIL if metrics.get("fall") else PASS
    collision_icon = PASS if metrics.get("collisions", 1) == 0 else FAIL

    rtf = metrics.get("rtf_mean")
    rtf_icon = PASS if rtf and rtf >= 0.95 else (WARN if rtf and rtf >= 0.7 else FAIL)

    lines = [
        f"# 导航测试报告: {scenario_name}",
        f"",
        f"**场景**: {scenario_desc}",
        f"**时间**: {timestamp}",
        f"**结果**: {status_icon} {'成功' if metrics.get('success') else '失败'} ({metrics.get('result_status', 'N/A')})",
        f"",
        f"## 指标总览",
        f"",
        f"| 指标 | 值 | 判定 | 说明 |",
        f"|------|-----|------|------|",
        f"| **摔倒** | {'是' if metrics.get('fall') else '否'} | {fall_icon} | z<{0.35}m 或 pitch/roll >45° |",
        f"| **碰撞** | {metrics.get('collisions', '?')} 次 | {collision_icon} | robot vs environment (排除地面) |",
        f"| **导航成功** | {'是' if metrics.get('success') else '否'} | {status_icon} | 未摔未撞且距离<0.35m |",
        f"| **位置误差** | {metrics.get('position_error_m', '?')} m | {PASS if (metrics.get('position_error_m') is not None and metrics['position_error_m'] < 0.35) else FAIL} | 真实位置 vs 目标 |",
        f"| **SLAM漂移** | mean={metrics.get('drift_mean_m', 'N/A')}m max={metrics.get('drift_max_m', 'N/A')}m p95={metrics.get('drift_p95_m', 'N/A')}m | {WARN if metrics.get('drift_mean_m', 0) and metrics.get('drift_mean_m', 0) > 0.3 else PASS} | FastLIO2 估计 vs ground truth (逐帧) |",
        f"| **规划时间** | {metrics.get('plan_time_s', 'N/A')} s | — | goal发出 → 首次有速度输出 |",
        f"| **完成时间** | {metrics.get('completion_time_s', '?')} s | — | sim_time |",
        f"| **路径效率** | {metrics.get('path_efficiency', 'N/A')} | {PASS if metrics.get('path_efficiency', 0) and metrics.get('path_efficiency', 0) > 0.6 else WARN} | 直线/实际 (1.0=完美) |",
        f"| **线速度Jerk** | {metrics.get('linear_jerk_rms', 'N/A')} m/s³ | {PASS if (metrics.get('linear_jerk_rms') is not None and metrics['linear_jerk_rms'] < 2.0) else WARN} | RMS, <2.0 为平滑 |",
        f"| **角速度Jerk** | {metrics.get('angular_jerk_rms', 'N/A')} rad/s³ | — | RMS |",
        f"| **方向反转** | {metrics.get('direction_reversals_per_sec', 'N/A')} /s | {PASS if (metrics.get('direction_reversals_per_sec') is not None and metrics['direction_reversals_per_sec'] < 2.0) else WARN} | <2.0/s 为稳定 |",
        f"| **速度范围** | vmax={metrics.get('vmax_m_s', 'N/A')} vmin={metrics.get('vmin_m_s', 'N/A')} vmean={metrics.get('vmean_m_s', 'N/A')} m/s | — | GT 轨迹差分 |",
        f"| **最小转弯半径** | {metrics.get('turning_radius_min_m', 'N/A')} m | — | GT 轨迹外接圆法 |",
        f"| **RTF** | {rtf} (min: {metrics.get('rtf_min', '?')}) | {rtf_icon} | ≥0.95 为实时 |",
        f"",
        f"## 测试参数",
        f"",
        f"- 目标: ({params['goal_x']:.1f}, {params['goal_y']:.1f}, yaw={math.degrees(params['goal_yaw']):.0f}°)",
        f"- 超时: {params['timeout']}s",
        f"- 超时触发: {'是' if metrics.get('timed_out') else '否'}",
        f"",
    ]

    return "\n".join(lines)


# ═══════════════════════════════════════════════════════════
#  CPU/内存采样 (可选)
# ═══════════════════════════════════════════════════════════

def start_pidstat(output_path: str) -> subprocess.Popen:
    """启动 pidstat 后台采样"""
    try:
        proc = subprocess.Popen(
            ["pidstat", "-ru", "1", "-C",
             "aimrt_main|mujoco_lidar_bridge|fastlio|nav2|component_container"],
            stdout=open(output_path, "w"),
            stderr=subprocess.DEVNULL,
        )
        return proc
    except FileNotFoundError:
        return None


def parse_pidstat(filepath: str) -> dict:
    """解析 pidstat 输出, 取各进程平均 CPU% 和峰值 RSS"""
    result = {}
    try:
        with open(filepath) as f:
            lines = f.readlines()
    except FileNotFoundError:
        return result

    # pidstat 格式: 多段, 每段有 Average: 行
    proc_stats = {}  # proc_name → {"cpu": [], "rss": []}
    for line in lines:
        parts = line.split()
        if len(parts) < 8:
            continue
        # CPU 段: UID PID ... %CPU %MEM ... Command
        # MEM 段: UID PID ... minflt/s majflt/s ... %MEM RSS Command
        cmd = parts[-1]
        if cmd in ("Average:", ""):
            continue
        if cmd not in proc_stats:
            proc_stats[cmd] = {"cpu": [], "rss": []}

        # 尝试提取 CPU% (倒数第 5 列附近)
        try:
            # CPU 段 (含 %CPU %MEM)
            if "%CPU" in line or parts[-3].replace(".", "").replace("-", "").isdigit():
                cpu_val = float(parts[-5])
                rss_val = float(parts[-2])
                if "kB" not in parts[-1] and cpu_val < 200:
                    proc_stats[cmd]["cpu"].append(cpu_val)
                if rss_val < 10_000_000:
                    proc_stats[cmd]["rss"].append(rss_val)
        except (ValueError, IndexError):
            pass

    for cmd, stats in proc_stats.items():
        cpus = stats["cpu"]
        rsss = stats["rss"]
        result[cmd] = {
            "cpu_mean_pct": round(sum(cpus) / len(cpus), 1) if cpus else 0,
            "cpu_max_pct": round(max(cpus), 1) if cpus else 0,
            "rss_peak_mb": round(max(rsss) / 1024, 1) if rsss else 0,
        }

    return result


# ═══════════════════════════════════════════════════════════
#  主流程
# ═══════════════════════════════════════════════════════════

def run_single_test(scenario_name: str, params: dict, report_dir: str) -> dict:
    """执行单个测试场景"""
    print(f"\n{'='*60}")
    print(f"  场景: {scenario_name} — {params['desc']}")
    print(f"  目标: ({params['goal_x']:.1f}, {params['goal_y']:.1f}, yaw={math.degrees(params['goal_yaw']):.0f}°)")
    print(f"  超时: {params['timeout']}s")
    print(f"{'='*60}\n")

    rclpy.init()
    node = NavTestNode(
        params["goal_x"], params["goal_y"], params["goal_yaw"], params["timeout"]
    )

    # CPU/内存采样
    pidstat_path = os.path.join(report_dir, f"{scenario_name}_pidstat.log")
    pidstat_proc = start_pidstat(pidstat_path)

    # 等待 ground truth 数据流 (5s 超时)
    print("[1/4] 等待 ground truth 数据流...")
    spin_start = time.monotonic()
    while len(node.metrics.gt_data) == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - spin_start > 10:
            print("  ❌ 10s 内未收到 /mujoco/ground_truth, 请确认 sim_module 已启动并切到 walk_mode")
            node.destroy_node()
            rclpy.shutdown()
            return {"scenario": scenario_name, "success": False, "error": "no_gt_data"}
    print("  ✓ ground truth 数据流正常")

    # 发送导航目标
    print("[2/4] 发送导航目标...")
    node.send_goal()

    # 等待完成或超时
    print(f"[3/4] 等待导航完成 (最长 {params['timeout']}s)...")
    while not node.finished:
        rclpy.spin_once(node, timeout_sec=0.5)
        # 进度显示
        if len(node.metrics.gt_data) > 0:
            latest = node.metrics.gt_data[-1]
            sim_t = latest[0]
            cx, cy = latest[1], latest[2]
            dist = math.sqrt((cx - params["goal_x"])**2 + (cy - params["goal_y"])**2)
            elapsed_wall = time.monotonic() - spin_start
            print(f"\r  sim_t={sim_t:.1f}s  pos=({cx:.2f},{cy:.2f})  dist={dist:.2f}m  wall={elapsed_wall:.0f}s",
                  end="", flush=True)

    print()  # 换行

    # 停止 pidstat
    if pidstat_proc:
        pidstat_proc.terminate()
        pidstat_proc.wait(timeout=3)

    # 计算指标
    print("[4/4] 计算指标...")
    metrics = node.metrics.compute_metrics(params["goal_x"], params["goal_y"], params["timeout"])
    metrics["result_status"] = node.result_status

    # CPU/内存
    cpu_mem = parse_pidstat(pidstat_path) if os.path.exists(pidstat_path) else {}
    metrics["cpu_mem"] = cpu_mem

    node.destroy_node()
    rclpy.shutdown()

    # 生成报告
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = os.path.join(report_dir, f"{scenario_name}_{timestamp}.md")
    json_path = os.path.join(report_dir, f"{scenario_name}_{timestamp}.json")

    report_md = format_report(scenario_name, params["desc"], params, metrics, timestamp)
    with open(report_path, "w") as f:
        f.write(report_md)

    full_result = {
        "scenario": scenario_name,
        "description": params["desc"],
        "timestamp": timestamp,
        "params": {
            "goal_x": params["goal_x"],
            "goal_y": params["goal_y"],
            "goal_yaw_deg": math.degrees(params["goal_yaw"]),
            "timeout": params["timeout"],
        },
        "metrics": metrics,
    }
    with open(json_path, "w") as f:
        json.dump(full_result, f, indent=2, ensure_ascii=False)

    # 控制台摘要
    print(f"\n{'─'*60}")
    print(f"  结果: {'✅ 成功' if metrics.get('success') else '❌ 失败'} ({metrics.get('result_status')})")
    print(f"  摔倒: {'是' if metrics.get('fall') else '否'}")
    print(f"  碰撞: {metrics.get('collisions', '?')} 次")
    print(f"  位置误差: {metrics.get('position_error_m', '?')} m")
    print(f"  SLAM漂移: mean={metrics.get('drift_mean_m', 'N/A')}m max={metrics.get('drift_max_m', 'N/A')}m")
    print(f"  规划时间: {metrics.get('plan_time_s', 'N/A')}s")
    print(f"  路径效率: {metrics.get('path_efficiency', 'N/A')}")
    print(f"  速度范围: vmax={metrics.get('vmax_m_s', 'N/A')} vmin={metrics.get('vmin_m_s', 'N/A')} vmean={metrics.get('vmean_m_s', 'N/A')} m/s")
    print(f"  最小转弯半径: {metrics.get('turning_radius_min_m', 'N/A')}m")
    print(f"  线Jerk: {metrics.get('linear_jerk_rms', 'N/A')} m/s³")
    print(f"  RTF: {metrics.get('rtf_mean', 'N/A')} (min: {metrics.get('rtf_min', 'N/A')})")
    print(f"  完成时间: {metrics.get('completion_time_s', '?')} s")
    if cpu_mem:
        print(f"  CPU/内存:")
        for proc, stats in cpu_mem.items():
            print(f"    {proc}: CPU {stats['cpu_mean_pct']}% (peak {stats['cpu_max_pct']}%), RSS {stats['rss_peak_mb']}MB")
    print(f"{'─'*60}")
    print(f"  报告: {report_path}")
    print(f"  JSON: {json_path}")
    print()

    return full_result


def main():
    parser = argparse.ArgumentParser(description="导航仿真自动化测试")
    parser.add_argument("--goal-x", type=float, default=5.0, help="目标 X (m)")
    parser.add_argument("--goal-y", type=float, default=0.0, help="目标 Y (m)")
    parser.add_argument("--goal-yaw", type=float, default=0.0, help="目标朝向 (rad)")
    parser.add_argument("--timeout", type=float, default=60, help="超时 (s)")
    parser.add_argument("--scenario", type=str, default=None,
                        help="预定义场景名 (A_straight_5m, C_narrow_passage, ...)")
    parser.add_argument("--batch", action="store_true", help="批量运行所有场景")
    parser.add_argument("--report-dir", type=str, default="reports",
                        help="报告输出目录")
    args = parser.parse_args()

    report_dir = os.path.abspath(args.report_dir)
    os.makedirs(report_dir, exist_ok=True)

    if args.batch:
        # 批量运行
        all_results = []
        for name, params in SCENARIOS.items():
            print(f"\n{'#'*60}")
            print(f"# 批量测试: {name}")
            print(f"{'#'*60}")

            result = run_single_test(name, params, report_dir)
            all_results.append(result)

            # 场景间等待
            print("\n  场景间等待 10s (确保系统稳定)...")
            time.sleep(10)

        # 汇总
        summary_path = os.path.join(report_dir, f"batch_summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
        with open(summary_path, "w") as f:
            json.dump(all_results, f, indent=2, ensure_ascii=False)

        # 控制台汇总表
        print(f"\n{'='*80}")
        print(f"  批量测试汇总 ({len(all_results)} 个场景)")
        print(f"{'='*80}")
        print(f"{'场景':<25} {'成功':<6} {'摔倒':<6} {'碰撞':<6} {'规划(s)':<8} {'漂移(m)':<10} {'vmax':<8} {'vmin':<8} {'R_min':<8}")
        print(f"{'─'*90}")
        for r in all_results:
            m = r.get("metrics", {})
            name = r["scenario"]
            succ = "✅" if m.get("success") else "❌"
            fall = "是" if m.get("fall") else "否"
            col = str(m.get("collisions", "?"))
            pt = str(m.get("plan_time_s", "?"))
            drift = str(m.get("drift_mean_m", "?"))
            vmax = str(m.get("vmax_m_s", "?"))
            vmin = str(m.get("vmin_m_s", "?"))
            rmin = str(m.get("turning_radius_min_m", "?"))
            print(f"{name:<25} {succ:<6} {fall:<6} {col:<6} {pt:<8} {drift:<10} {vmax:<8} {vmin:<8} {rmin:<8}")
        print(f"{'─'*90}")
        print(f"  汇总: {summary_path}\n")

    elif args.scenario:
        # 预定义场景
        if args.scenario not in SCENARIOS:
            print(f"未知场景: {args.scenario}")
            print(f"可用: {', '.join(SCENARIOS.keys())}")
            sys.exit(1)
        params = SCENARIOS[args.scenario]
        run_single_test(args.scenario, params, report_dir)

    else:
        # 自定义单次测试
        params = {
            "desc": f"自定义 ({args.goal_x:.1f}, {args.goal_y:.1f})",
            "goal_x": args.goal_x,
            "goal_y": args.goal_y,
            "goal_yaw": args.goal_yaw,
            "timeout": int(args.timeout),
        }
        run_single_test("custom", params, report_dir)


if __name__ == "__main__":
    main()
