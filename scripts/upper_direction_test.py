#!/usr/bin/env python3
"""Prepare and run one-joint-at-a-time static upper-body direction checks.

The ROS client only publishes mode events. The existing C++ PD controller owns
joint commands and its 2 s interpolation. Physical direction is operator input.
"""

from __future__ import annotations

import argparse
import copy
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import select
import signal
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
MODEL = ROOT / "motion_control/module/sim_module/model/mjcf/robot/xyber_x1/xyber_x1_serial.xml"
CONTROL_REL = Path("cfg/control_module/upper_direction_test.yaml")
MAIN_REL = Path("cfg/x1_cfg_upper_direction_test.yaml")
MANIFEST_REL = Path("cfg/control_module/upper_direction_test.json")
PREFIX = "/upper_static/"
FIELDS = ("init_state", "stiffness", "damping")

# Signs stay inside the deployed limits. Shoulder roll's zero is its upper bound.
JOINTS = [
    ("lumbar_yaw_joint", "腰 yaw", 1, "上半身向机器人左侧转（从上向下看逆时针）"),
    ("lumbar_roll_joint", "腰 roll", 1, "上半身相对骨盆向机器人左侧折"),
    ("lumbar_pitch_joint", "腰 pitch", 1, "上半身相对骨盆向前折（复查已修正的右腰方向）"),
]
for _side, _label in (("left", "左"), ("right", "右")):
    JOINTS += [
        (f"{_side}_shoulder_pitch_joint", f"{_label}肩 pitch", 1, "中立垂臂时上臂向后摆"),
        (f"{_side}_shoulder_roll_joint", f"{_label}肩 roll", -1, "负角度使上臂向身体外侧展开"),
        (f"{_side}_shoulder_yaw_joint", f"{_label}肩 yaw", 1,
         "绕上臂轴旋转；从上向下看" + ("逆时针" if _side == "left" else "顺时针")),
        (f"{_side}_elbow_pitch_joint", f"{_label}肘 pitch", 1, "中立垂臂时前臂向前折"),
        (f"{_side}_elbow_yaw_joint", f"{_label}肘 yaw", 1,
         "绕前臂轴旋转；从上向下看" + ("逆时针" if _side == "left" else "顺时针") + "，直臂时看表面标记"),
        (f"{_side}_wrist_pitch_joint", f"{_label}腕 pitch", 1,
         "中立位绕机器人" + ("+X" if _side == "left" else "-X") + "轴按右手定则旋转（从前方看" +
         ("逆时针" if _side == "left" else "顺时针") + "）"),
        (f"{_side}_wrist_roll_joint", f"{_label}腕 roll", 1,
         "中立位绕机器人 +Y 轴按右手定则旋转（从机器人左侧看逆时针）"),
    ]
UPPER = {joint for joint, *_ in JOINTS}
LOWER = {f"{side}_{kind}_joint" for side in ("left", "right")
         for kind in ("hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch", "ankle_roll")}


def load_config(path):
    text = Path(path).read_text(encoding="utf-8")
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml
        except ImportError as error:
            raise ValueError("读取 YAML 需要 python3-yaml；请使用 ROS2 的 Python 环境。") from error
        value = yaml.safe_load(text)
    if not isinstance(value, dict):
        raise ValueError(f"配置必须是映射：{path}")
    return value


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"需要有限数值：{value!r}")
    return float(value)


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotate(r, v):
    return [sum(r[i][k] * v[k] for k in range(3)) for i in range(3)]


def quaternion_matrix(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-12:
        raise ValueError("模型含无效四元数")
    w, x, y, z = [v / norm for v in q]
    return [[1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
            [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
            [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)]]


def model_axes(model, neutral):
    """Joint axes in the pelvis frame, including parents' neutral joint angles."""
    axes = {}
    body = ET.parse(model).find("worldbody/body")
    if body is None:
        raise ValueError("模型缺少机器人根 body")

    def visit(element, parent_rotation):
        if any(key in element.attrib for key in ("euler", "axisangle", "xyaxes", "zaxis")):
            raise ValueError("参考模型 body 应使用 quat；不能忽略其旋转")
        rotation = matmul(parent_rotation, quaternion_matrix(
            [float(v) for v in element.get("quat", "1 0 0 0").split()]))
        for joint in element.findall("joint"):
            name = joint.get("name")
            local = [float(v) for v in joint.get("axis", "0 0 1").split()]
            length = math.sqrt(sum(v*v for v in local))
            local = [v / length for v in local]
            axes[name] = rotate(rotation, local)
            half = neutral.get(name, 0.0) / 2
            rotation = matmul(rotation, quaternion_matrix(
                [math.cos(half), *(v * math.sin(half) for v in local)]))
        for child in element.findall("body"):
            visit(child, rotation)

    visit(body, [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    if not UPPER <= axes.keys():
        raise ValueError("参考模型缺少上肢关节")
    return axes


def actuator_bindings(dcu):
    bindings = {}
    for trans in dcu["transmission"]:
        if trans["type"] == "SimpleTransmission":
            pairs = [(trans["joint"], trans["actuator"], trans["direction"])]
        elif trans["type"] in ("LumbarParallelTransmission", "LeftWristParallelTransmission",
                               "RightWristParallelTransmission"):
            pairs = [(trans["joint_pitch"], trans["actuator_right"], trans["direction_right"]),
                     (trans["joint_roll"], trans["actuator_left"], trans["direction_left"])]
        else:
            continue
        for joint, actuator, direction in pairs:
            if joint not in UPPER:
                continue
            direction = number(direction)
            if direction not in (-1, 1) or joint in bindings:
                raise ValueError(f"上肢传动方向/绑定无效：{joint}")
            bindings[joint] = {"actuator": actuator, "direction": direction,
                               "transmission": trans["name"]}
    if bindings.keys() != UPPER:
        raise ValueError("DCU 上肢关节绑定不完整")
    return bindings


def build_configs(control, main, dcu, angle=0.10, model=MODEL):
    """Return isolated PD configs; never mutate the supplied live configs."""
    angle = number(angle)
    if not 0 < angle <= 0.20:
        raise ValueError("测试角度应为 (0, 0.20] rad；默认 0.10 rad")
    if control.get("use_sim_handles") is not False or control["control_frequecy"] != 1000:
        raise ValueError("此工具使用真机 1000 Hz PD 配置")
    names = control["joint_list"]
    lower = [name for name in names if name not in UPPER]
    if len(names) != 29 or set(names) != UPPER | LOWER:
        raise ValueError("需要 X1 的 17 个上肢关节和 12 个下肢关节")
    if set(dcu["joint_list"]) != set(names):
        raise ValueError("DCU 与控制器关节集不一致")
    pose = {}
    for controller_name in control["robot_states"]["stand"]["controllers"]:
        controller = control["controllers"][controller_name]
        if not controller_name.startswith("pd_") or controller.get("is_keep_controller") or controller.get("plan_conf"):
            raise ValueError("stand 必须由静态 PD 控制器组成")
        joint_names = controller["joint_list"]
        if len(set(joint_names)) != len(joint_names) or any(len(controller[f]) != len(joint_names) for f in FIELDS):
            raise ValueError(f"PD 数组长度或关节名称无效：{controller_name}")
        for i, name in enumerate(joint_names):
            pose[name] = [number(controller[field][i]) for field in FIELDS]
    if set(pose) != set(names):
        raise ValueError("stand 未覆盖全部关节")
    if any(not (0 <= values[1] <= 500 and 0 <= values[2] <= 8) for values in pose.values()):
        raise ValueError("真机静态 kp/kd 超出 MIT 编码范围")
    neutral = {name: values[0] for name, values in pose.items()}
    for name, target in neutral.items():
        limits = control["joint_limits"][name]
        if not number(limits["lower"]) <= target <= number(limits["upper"]):
            raise ValueError(f"stand 目标超限：{name}")
        number(control["joint_offset"][name])
    axes = model_axes(model, neutral)
    bindings = actuator_bindings(dcu)
    base = {"joint_list": list(names), **{
        field: [pose[name][i] for name in names] for i, field in enumerate(FIELDS)}}
    idle = copy.deepcopy(control["controllers"]["pd_idle"])
    if set(idle["joint_list"]) != set(names) or any(any(number(v) != 0 for v in idle[field])
                                                  for field in ("stiffness", "damping")):
        raise ValueError("pd_idle 应为全关节零增益")
    keep = copy.deepcopy(base)
    keep["is_keep_controller"] = True
    output = copy.deepcopy(control)
    output["initial_state"] = "idle"
    output["controllers"] = {"pd_idle": idle, "pd_neutral": base, "pd_keep": keep}
    cases = []
    for name, label, sign, expected in JOINTS:
        target = neutral[name] + sign * angle
        limits = control["joint_limits"][name]
        if not limits["lower"] <= target <= limits["upper"]:
            raise ValueError(f"测试目标超限：{name} -> {target:+.4f}")
        key = name.removesuffix("_joint")
        controller = copy.deepcopy(base)
        controller["init_state"][names.index(name)] = target
        output["controllers"]["pd_test_" + key] = controller
        binding = bindings[name]
        cases.append({"joint": name, "label": label, "delta": sign * angle,
                      "neutral": neutral[name], "target": target,
                      "topic": PREFIX + key, "state": "test_" + key,
                      "expected": expected, "positive_axis": axes[name], **binding,
                      "native_target": binding["direction"] * (target + control["joint_offset"][name])})
    test_states = [case["state"] for case in cases]
    states = {
        "idle": {"trigger_topic": PREFIX + "idle", "pre_states": ["neutral", "keep", *test_states],
                 "controllers": ["pd_idle"]},
        "neutral": {"trigger_topic": PREFIX + "neutral", "pre_states": ["idle", "keep", *test_states],
                    "controllers": ["pd_neutral"]},
        "keep": {"trigger_topic": PREFIX + "keep", "pre_states": ["idle", "neutral", *test_states],
                 "controllers": ["pd_keep"]},
    }
    for case in cases:
        states[case["state"]] = {"trigger_topic": case["topic"], "pre_states": ["neutral"],
                                 "controllers": ["pd_" + case["state"]]}
    output["robot_states"] = states
    main_output = copy.deepcopy(main)
    aimrt = main_output["aimrt"]
    for plugin in aimrt["plugin"]["plugins"]:
        if plugin["name"] == "ros2_plugin":
            plugin["options"]["node_name"] = "x1_upper_static_ctrl"
    for backend in aimrt["log"]["backends"]:
        if backend["type"] == "rotate_file":
            backend["options"]["filename"] = "x1_upper_static.log"
    for package in aimrt["module"]["pkgs"]:
        package["enable_modules"] = [name for name in package["enable_modules"]
                                     if name in ("ControlModule", "DcuDriverModule")]
    if {name for package in aimrt["module"]["pkgs"] for name in package["enable_modules"]} != {"ControlModule", "DcuDriverModule"}:
        raise ValueError("主配置动态库必须加载 ControlModule 与 DcuDriverModule")
    modules = aimrt["module"]["modules"]
    aimrt["module"]["modules"] = [m for m in modules if m["name"] in ("ControlModule", "DcuDriverModule")]
    if {m["name"] for m in aimrt["module"]["modules"]} != {"ControlModule", "DcuDriverModule"}:
        raise ValueError("主配置需要 ControlModule 与 DcuDriverModule")
    for module in aimrt["module"]["modules"]:
        if module["name"] == "ControlModule":
            module["cfg_file_path"] = "./" + str(CONTROL_REL)
    aimrt["channel"]["sub_topics_options"].insert(0, {
        "topic_name": PREFIX + ".*", "enable_backends": ["local", "ros2"]})
    aimrt["channel"]["pub_topics_options"].insert(0, {
        "topic_name": "/(joint_cmd|joint_states)", "enable_backends": ["local", "ros2"]})
    plan = {"version": 1, "angle": angle, "joint_names": names, "lower_joints": lower,
            "neutral": neutral, "offsets": control["joint_offset"], "cases": cases,
            "stiffness": dict(zip(names, base["stiffness"])),
            "damping": dict(zip(names, base["damping"])), "feedback_topic": control["sub_joint_state_name"],
            "command_topic": control["pub_joint_cmd_name"], "interpolation_seconds": 2.0}
    if plan["feedback_topic"] != "/joint_states" or plan["command_topic"] != "/joint_cmd":
        raise ValueError("此工具要求标准 /joint_states 和 /joint_cmd 话题")
    return output, main_output, plan


def print_cases(plan, cases=None):
    print("坐标：X 前、Y 左、Z 上；方向说明以模型中立垂臂姿态为参考。")
    for index, case in enumerate(cases or plan["cases"], 1):
        axis = ", ".join(f"{v:+.3f}" for v in case["positive_axis"])
        print(f"{index:2d}. {case['joint']}: {case['neutral']:+.3f} -> {case['target']:+.3f} rad "
              f"(Δ{math.degrees(case['delta']):+.1f}°)")
        print(f"    {case['expected']}；正角度旋转轴 ({axis})")


def resolve_config(bin_dir, path):
    return Path(path) if Path(path).is_absolute() else bin_dir / path


def prepare(args):
    bin_dir = args.bin_dir.resolve()
    main_path = bin_dir / "cfg/x1_cfg_real_nav.yaml"
    main = load_config(main_path)
    modules = {m["name"]: m for m in main["aimrt"]["module"]["modules"]}
    control_path = resolve_config(bin_dir, modules["ControlModule"]["cfg_file_path"])
    dcu_path = resolve_config(bin_dir, modules["DcuDriverModule"]["cfg_file_path"])
    control, main_output, plan = build_configs(load_config(control_path), main, load_config(dcu_path), args.angle)
    plan["bin_dir"] = str(bin_dir)
    plan["sources"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in (main_path, control_path, dcu_path, MODEL)}
    print_cases(plan)
    if args.dry_run:
        print("预览结束，未写配置、未发送机器人指令。")
        return
    encoded = json.dumps(control, ensure_ascii=False, indent=2) + "\n"
    # JSON is also valid YAML; yaml-cpp reads these generated files directly.
    control_file = bin_dir / CONTROL_REL
    control_file.parent.mkdir(parents=True, exist_ok=True)
    control_file.write_text(encoded, encoding="utf-8")
    plan["control_sha256"] = hashlib.sha256(encoded.encode()).hexdigest()
    main_encoded = json.dumps(main_output, ensure_ascii=False, indent=2) + "\n"
    (bin_dir / MAIN_REL).write_text(main_encoded, encoding="utf-8")
    plan["main_sha256"] = hashlib.sha256(main_encoded.encode()).hexdigest()
    (bin_dir / MANIFEST_REL).write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"已生成 {bin_dir / MAIN_REL}；原 AMP/DCU 配置文件未改写。")


def read_positions(msg, names):
    if len(msg.name) != len(set(msg.name)) or len(msg.position) != len(msg.name):
        raise ValueError("关节消息名称重复或位置数组长度不一致")
    values = dict(zip(msg.name, msg.position))
    if not set(names) <= values.keys():
        raise ValueError("关节消息缺少所需关节")
    return {name: number(values[name]) for name in names}


def command_matches(command, desired, offsets, tolerance=1e-4):
    return command is not None and all(abs(command[name] - offsets[name] - desired[name]) <= tolerance
                                       for name in desired)


def choose_cases(plan, joints):
    by_name = {case["joint"]: case for case in plan["cases"]}
    if joints:
        if len(set(joints)) != len(joints) or any(name not in by_name for name in joints):
            raise ValueError("--joint 必须是不重复的上肢关节名；下肢不能选择")
        return [by_name[name] for name in joints]
    return plan["cases"]


def run(args):
    # Imports are lazy so prepare/list/unit tests work without ROS2.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32
    from my_ros2_proto.msg import JointCommand

    plan = load_config(args.manifest)
    cases = choose_cases(plan, args.joint)
    active = Path(plan["bin_dir"]) / CONTROL_REL
    if hashlib.sha256(active.read_bytes()).hexdigest() != plan["control_sha256"]:
        raise ValueError("静态控制配置与测试清单不一致，请重新 prepare 并重启")
    main_file = Path(plan["bin_dir"]) / MAIN_REL
    if hashlib.sha256(main_file.read_bytes()).hexdigest() != plan["main_sha256"]:
        raise ValueError("静态主配置与测试清单不一致，请重新 prepare 并重启")
    for path, digest in plan["sources"].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise ValueError(f"准备测试后源配置发生变化：{path}；请重新 prepare 并重启")
    if not sys.stdin.isatty():
        raise ValueError("逐关节测试需要交互终端")
    log_dir = Path(plan["bin_dir"]) / "test_logs/data_csv"
    log_dir.mkdir(parents=True, exist_ok=True)
    stem = "upper_direction_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    csv_path = log_dir / (stem + ".csv")
    report_path = log_dir / (stem + ".json")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("upper_direction_test_operator")
    names, offsets = plan["joint_names"], plan["offsets"]
    latest = {"command": None, "feedback": None, "command_at": 0.0, "feedback_at": 0.0, "error": None}
    phase = "waiting"
    start_time = time.monotonic()
    results = []
    active_case = None
    tracking = []
    logging_enabled = True
    stable_targets = None
    csv_stream = csv_path.open("w", encoding="utf-8", newline="")
    fields = ["elapsed_s", "phase", "test_joint"] + [f"target_{n}" for n in names] + [f"position_{n}" for n in names]
    writer = csv.DictWriter(csv_stream, fieldnames=fields)
    writer.writeheader()

    def receive(msg, key):
        try:
            latest[key] = read_positions(msg, names)
            latest[key + "_at"] = time.monotonic()
        except ValueError as error:
            latest["error"] = str(error)

    node.create_subscription(JointCommand, plan["command_topic"], lambda m: receive(m, "command"), qos_profile_sensor_data)
    node.create_subscription(JointState, plan["feedback_topic"], lambda m: receive(m, "feedback"), qos_profile_sensor_data)
    pubs = {topic: node.create_publisher(Float32, topic, 10) for topic in
            [PREFIX + "neutral", PREFIX + "keep", PREFIX + "idle", *(c["topic"] for c in cases)]}

    def log_sample():
        if not logging_enabled or latest["command"] is None or latest["feedback"] is None:
            return
        row = {"elapsed_s": f"{time.monotonic() - start_time:.3f}", "phase": phase,
               "test_joint": active_case["joint"] if active_case else ""}
        row.update({f"target_{n}": latest["command"][n] - offsets[n] for n in names})
        row.update({f"position_{n}": latest["feedback"][n] - offsets[n] for n in names})
        writer.writerow(row)
        if phase == "observe" and active_case:
            joint = active_case["joint"]
            tracking.append(latest["feedback"][joint] - offsets[joint])

    node.create_timer(0.04, log_sample)

    def healthy():
        now = time.monotonic()
        if latest["error"]:
            raise RuntimeError(latest["error"])
        if any(latest[key] is None or now - latest[key + "_at"] > 0.5 for key in ("command", "feedback")):
            raise RuntimeError("关节目标或反馈超过 0.5 秒未更新")
        if node.count_publishers(plan["command_topic"]) != 1:
            raise RuntimeError("/joint_cmd 必须只有静态控制器一个发布者")
        if node.count_publishers(plan["feedback_topic"]) != 1:
            raise RuntimeError("/joint_states 必须只有真机 DCU 一个发布者")
        if phase in ("ready", "observe", "finished_neutral") and stable_targets is not None:
            if not command_matches(latest["command"], stable_targets, offsets):
                raise RuntimeError("已稳定的静态目标被其他模式改变")

    def tick(check=True):
        rclpy.spin_once(node, timeout_sec=0.01)
        if check:
            healthy()

    def wait(seconds, check=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            tick(check)

    def prompt(text):
        print(text, end="", flush=True)
        while True:
            tick()
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                value = sys.stdin.readline()
                if not value:
                    raise KeyboardInterrupt
                return value.strip().lower()

    def trigger(topic, check=True):
        deadline = time.monotonic() + 2.2
        next_pub = 0.0
        msg = Float32()
        msg.data = 1.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_pub:
                pubs[topic].publish(msg)
                next_pub = now + 0.2
            tick(check)

    def reach(topic, targets):
        nonlocal stable_targets
        trigger(topic)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            tick()
            if command_matches(latest["command"], targets, offsets):
                stable_targets = dict(targets)
                return
        raise RuntimeError("目标未到达所选静态姿态；检查模式转换日志")

    report = {"plan": str(args.manifest.resolve()), "csv": str(csv_path), "results": results,
              "physical_direction_source": "operator observation; joint feedback alone cannot prove direction",
              "completed": False}
    armed = False
    def terminated(signum, frame):
        raise KeyboardInterrupt
    previous_term_handler = signal.signal(signal.SIGTERM, terminated)
    try:
        print(f"日志：{csv_path}\n请先支撑机器人；此测试不加载手柄或行走策略。")
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            tick(False)
            if latest["command"] is not None and latest["feedback"] is not None and node.count_subscribers(PREFIX + "neutral"):
                healthy()
                break
        else:
            raise RuntimeError("未发现静态控制入口和全身反馈；请先运行 start，不能连接普通 run.sh")
        wait(1.1)
        print_cases(plan, cases)
        while True:
            choice = prompt("回车进入静态 stand 基准；q 退出：")
            if choice in ("", "q"):
                break
        if choice == "q": return
        armed = True
        phase = "prepare_neutral"
        reach(PREFIX + "neutral", plan["neutral"])
        wait(1.0)
        for case in cases:
            active_case = case
            phase = "ready"
            axis = ", ".join(f"{v:+.3f}" for v in case["positive_axis"])
            print(f"\n{case['label']} / {case['joint']}，Δ{case['delta']:+.3f} rad")
            print(f"预期：{case['expected']}；正角度轴 ({axis})")
            print(f"电机：{case['actuator']}，direction={case['direction']:+.0f}，目标 {case['native_target']:+.3f} rad")
            while True:
                choice = prompt("回车开始；s 跳过；q 结束：")
                if choice in ("", "s", "q"):
                    break
            if choice == "q":
                break
            if choice == "s":
                results.append({"joint": case["joint"], "observation": "skipped"})
                continue
            tracking = []
            targets = dict(plan["neutral"])
            targets[case["joint"]] = case["target"]
            phase = "ramp_test"
            reach(case["topic"], targets)
            phase = "observe"
            print("目标插值完成，保持供观察；等姿态稳定后输入分类，随后自动返回基准目标。")
            while True:
                observation = prompt("s=方向一致，r=方向相反，u=不明显，w=实际动了别的关节，q=结束：")
                if observation in ("s", "r", "u", "w", "q"):
                    break
            samples = list(tracking)
            phase = "return_neutral"
            reach(PREFIX + "neutral", plan["neutral"])
            wait(0.5)
            result = {"joint": case["joint"], "target": case["target"], "delta": case["delta"],
                      "observation": {"s": "same", "r": "opposite", "u": "unclear", "w": "wrong_joint", "q": "aborted"}[observation],
                      "samples": len(samples), "actuator": case["actuator"], "direction": case["direction"]}
            if samples:
                result.update({"feedback_mean": sum(samples) / len(samples),
                               "tracking_mae": sum(abs(v - case["target"]) for v in samples) / len(samples)})
            results.append(result)
            csv_stream.flush()
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print("目标已恢复静态基准。")
            if observation == "q":
                break
        else:
            report["completed"] = True
        active_case = None
        phase = "finished_neutral"
        print("测试结束，保持静态 stand 基准目标；在启动终端 Ctrl+C 关闭测试控制进程。")
    except (KeyboardInterrupt, Exception) as error:
        report["error"] = str(error) or "operator interrupt"
        phase = "abort"
        logging_enabled = False
        print(f"\n测试中止：{report['error']}")
        if armed:
            # Capture a fresh measured pose if available; stale feedback calls for zero gains.
            topic = PREFIX + ("keep" if time.monotonic() - latest["feedback_at"] <= 0.5 else "idle")
            print(f"发送 {topic}；硬件急停仍由现场操作。")
            try:
                trigger(topic, check=False)
            except (Exception, KeyboardInterrupt) as stop_error:
                print(f"软件中止指令未完成：{stop_error}；请在现场停止控制。")
        raise RuntimeError(report["error"]) from error
    finally:
        signal.signal(signal.SIGTERM, previous_term_handler)
        try:
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            csv_stream.close()
            print(f"结果：{report_path}")
        except OSError as log_error:
            print(f"结果文件写入失败：{log_error}")
        finally:
            node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prep = commands.add_parser("prepare", help="从已安装的现场配置生成独立静态测试配置")
    prep.add_argument("--bin-dir", type=Path, default=ROOT / "build")
    prep.add_argument("--angle", type=float, default=0.10)
    prep.add_argument("--dry-run", action="store_true")
    for name in ("list", "run"):
        sub = commands.add_parser(name)
        sub.add_argument("--manifest", type=Path, default=ROOT / "build" / MANIFEST_REL)
        sub.add_argument("--joint", action="append", help="只选择此上肢关节；可重复指定")
    args = parser.parse_args()
    try:
        if args.command == "prepare":
            prepare(args)
        elif args.command == "list":
            plan = load_config(args.manifest)
            print_cases(plan, choose_cases(plan, args.joint))
        else:
            run(args)
    except (OSError, ValueError, KeyError, RuntimeError, ImportError) as error:
        parser.exit(1, f"错误：{error}\n")


if __name__ == "__main__":
    main()
