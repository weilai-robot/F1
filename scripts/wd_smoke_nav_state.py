#!/usr/bin/env python3
"""wd_smoke v2 — nav_state_manager 纯看门狗冒烟回归（2026-10-16 复审补丁版）

用法（WSL / 机器人侧均可，需 ROS2 环境）：
    source /opt/ros/humble/setup.bash && python3 F1/scripts/wd_smoke_nav_state.py
    # 可选：python3 F1/scripts/wd_smoke_nav_state.py <nav_state_manager.py 绝对路径>
    # 耗时 ~35s；节点子进程日志落 /tmp/wd_smoke_node.log（不入仓的工件）

⚠️ 纪律：**每次改 nav_state_manager.py 之后必须重跑本脚本**（P0 校验 V0 的
教训——2026-10-16 实施者留档冒烟跑的是上一次编辑的代码，最后一轮文案级
编辑未被留档覆盖，靠校验方重跑才闭合证明链）。改判据/触发逻辑不重跑 =
回归结论无效。

相位（设计见 docs/nav/改进TODO/P0-nav-state-看门狗改造方案.md §10）：
  1) 线② limiter 心跳源：/cmd_vel 与 /cmd_vel_limiter 零值 10Hz 流动期
     不触发（零值=心跳，P1 等待语义不误杀）；双路沉默 >3s → burst1
     （判定源=limiter，节点日志应写 "/cmd_vel_limiter 沉默"）
  2) 线① + F1 re-arm：第二 goal SUCCEEDED → 0.5s 后 burst2（无任何心跳
     消息，验旧一次性缺陷已修：第二个 goal 到点必须再发 stand）
  3) odom_bridge 死场景（P4 新覆盖）：cmd_vel 非零持续流动、limiter 沉默
     → 3s 后 burst3——旧判据（盯 /cmd_vel）在此场景永远不触发

断言：共 30 条 /stand_mode = 3 burst ×10（5Hz×2s），起点落位三窗口，
零值心跳期零触发，节点兜底日志 ≥2 次指名 limiter。相位 2 会出一条
「nav_active 已 Ns 但从未收到」WARN——已知合成噪音（驱动脚本人为拼接
"线② 止停无终态→新 goal"序列所致，真机 goal 必有终态下降沿重置计时，
见校验报告 F-3），非缺陷。
"""
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# 默认相对本文件解析（F1/scripts/ → 仓内 navigation 子仓）；argv[1] 可覆盖
NODE_SCRIPT = (
    sys.argv[1] if len(sys.argv) > 1 else
    str(Path(__file__).resolve().parent.parent
        / 'navigation' / 'planning' / 'humanoid_sim' / 'scripts'
        / 'nav_state_manager.py'))
NODE_LOG = "/tmp/wd_smoke_node.log"
T0 = time.monotonic()
events = []
lock = threading.Lock()


class Smoke(Node):
    def __init__(self):
        super().__init__('wd_smoke')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lim_pub = self.create_publisher(Twist, '/cmd_vel_limiter', 10)
        self.walk_pub = self.create_publisher(Float32, '/walk_mode', 10)
        self.sta_pub = self.create_publisher(
            GoalStatusArray, '/navigate_to_pose/_action/status', 10)
        self.sub = self.create_subscription(
            Float32, '/stand_mode', self.on_stand, 10)

    def on_stand(self, msg):
        with lock:
            events.append(time.monotonic() - T0)

    def pub_walk(self):
        # 真机口径 data=0.0（事件推断忽略 data）
        m = Float32()
        m.data = 0.0
        self.walk_pub.publish(m)

    def pub_walk_burst(self):
        # 5Hz×2s 语义的浓缩：3 连发盖 DDS 发现窗
        for _ in range(3):
            self.pub_walk()
            time.sleep(0.2)

    def pub_status(self, status):
        m = GoalStatusArray()
        gs = GoalStatus()
        gs.status = status
        m.status_list = [gs]
        self.sta_pub.publish(m)

    def pub_status_twice(self, status):
        self.pub_status(status)
        time.sleep(0.25)
        self.pub_status(status)

    def pub_twist(self, pub, vx):
        m = Twist()
        m.linear.x = vx
        pub.publish(m)


def wait_to(t):
    d = t - (time.monotonic() - T0)
    if d > 0:
        time.sleep(d)


def drive(node, done):
    # ── 相位 1：线② limiter 心跳 ──
    wait_to(1.0)
    node.pub_walk_burst()                       # walk 事件 → in_walk + re-arm
    wait_to(2.0)
    node.pub_status_twice(GoalStatus.STATUS_EXECUTING)
    for i in range(29):                         # 2.1→5.0 双路零值 10Hz
        wait_to(2.1 + i * 0.1)
        node.pub_twist(node.cmd_pub, 0.0)
        node.pub_twist(node.lim_pub, 0.0)
    # 5.0 起双路沉默 → 期待 ~8.3 burst1

    # ── 相位 2：线① + F1 re-arm（无心跳消息）──
    wait_to(13.5)
    node.pub_walk_burst()                       # re-arm
    wait_to(14.5)
    node.pub_status_twice(GoalStatus.STATUS_EXECUTING)
    wait_to(15.0)
    node.pub_status_twice(GoalStatus.STATUS_SUCCEEDED)   # → burst2 ~15.5

    # ── 相位 3：odom_bridge 死（cmd_vel 活、limiter 停）──
    wait_to(20.2)
    node.pub_walk_burst()
    wait_to(21.2)
    node.pub_status_twice(GoalStatus.STATUS_EXECUTING)
    for i in range(21):                         # 21.2→23.2 双路流（cmd 非零）
        wait_to(21.2 + i * 0.1)
        node.pub_twist(node.cmd_pub, 0.3)
        node.pub_twist(node.lim_pub, 0.0)
    for i in range(70):                         # 23.2→30.2 仅 cmd_vel 活
        wait_to(23.2 + i * 0.1)
        node.pub_twist(node.cmd_pub, 0.3)
    # 期待 ~26.3 burst3
    done.set()


def main():
    if not Path(NODE_SCRIPT).is_file():
        print(f"[smoke] FAIL: 找不到被测节点 {NODE_SCRIPT}")
        sys.exit(2)

    log = open(NODE_LOG, 'w')
    proc = subprocess.Popen([sys.executable, NODE_SCRIPT],
                            stdout=log, stderr=subprocess.STDOUT)
    time.sleep(1.0)                             # 节点先起（订阅就位）
    rclpy.init()
    node = Smoke()
    done = threading.Event()
    threading.Thread(target=drive, args=(node, done), daemon=True).start()

    deadline = time.monotonic() + 29.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
    log.close()

    # ── 断言 ──
    with lock:
        ev = sorted(events)
    fails = []
    print(f"[smoke] /stand_mode 共 {len(ev)} 条: {[round(t,1) for t in ev]}")

    if len(ev) != 30:
        fails.append(f"消息数 {len(ev)} != 30（每 burst 10×3）")

    # 按 0.5s 间隙聚簇
    bursts = []
    for t in ev:
        if bursts and t - bursts[-1][-1] < 0.5:
            bursts[-1].append(t)
        else:
            bursts.append([t])
    print(f"[smoke] burst 起点数={len(bursts)}: "
          f"{[ (round(b[0],1), round(b[-1],1)) for b in bursts ]}")

    if len(bursts) != 3:
        fails.append(f"burst 数 {len(bursts)} != 3")
    else:
        b1, b2, b3 = bursts
        if not (7.0 <= b1[0] <= 9.5):
            fails.append(f"burst1 起点 {b1[0]:.1f} 不在 [7.0,9.5]（线② 沉默触发）")
        if not (14.9 <= b2[0] <= 16.2):
            fails.append(f"burst2 起点 {b2[0]:.1f} 不在 [14.9,16.2]（线①+re-arm）")
        if not (25.5 <= b3[0] <= 28.0):
            fails.append(f"burst3 起点 {b3[0]:.1f} 不在 [25.5,28.0]（odom_bridge 死）")

    if ev and ev[0] < 6.5:
        fails.append(f"零值心跳期出现早期触发（{ev[0]:.1f}）——误杀")

    # 节点日志：burst1/burst3 的兜底日志应指名 /cmd_vel_limiter 沉默
    logtxt = open(NODE_LOG, errors='replace').read()
    n_limiter = logtxt.count('cmd_vel_limiter 沉默')
    print(f"[smoke] 节点日志含 'cmd_vel_limiter 沉默' ×{n_limiter}（期待 ≥2：burst1+burst3）")
    if n_limiter < 2:
        fails.append(f"日志 'cmd_vel_limiter 沉默' 出现 {n_limiter} 次 <2")

    if fails:
        print("[smoke] FAIL:")
        for f in fails:
            print(f"  - {f}")
        sys.exit(1)
    print("[smoke] PASS（线②limiter 心跳不误杀 + 沉默触发 + 线①re-arm 二次触发"
          " + odom_bridge 死场景兜底）")


if __name__ == '__main__':
    main()
