"""Offline checks for generated PD modes and the interactive ROS client contract."""

import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import types
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "upper_direction_test.py"
SPEC = importlib.util.spec_from_file_location("upper_direction_test", SCRIPT)
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def inputs():
    upper = [name for name, *_ in tool.JOINTS]
    lower = [f"{side}_{kind}_joint" for side in ("left", "right")
             for kind in ("hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch", "ankle_roll")]
    names = upper + lower
    zero = {"joint_list": list(reversed(names)), "init_state": [0.0] * 29,
            "stiffness": [150.0 if name.startswith("lumbar") else 30.0 for name in reversed(names)],
            "damping": [5.0 if name.startswith("lumbar") else 1.5 for name in reversed(names)]}
    stand = {"joint_list": lower + ["left_shoulder_pitch_joint"],
             "init_state": [0.4 if "knee" in name else 0.0 for name in lower] + [0.0],
             "stiffness": [300.0] * 12 + [55.0], "damping": [5.0] * 12 + [2.0]}
    idle = copy.deepcopy(zero)
    idle["stiffness"] = idle["damping"] = [0.0] * 29
    limits = {name: {"lower": -1.0, "upper": 1.0} for name in names}
    for side in ("left", "right"):
        limits[f"{side}_shoulder_roll_joint"]["upper"] = 0.0
        limits[f"{side}_elbow_pitch_joint"]["lower"] = 0.0
    limits["lumbar_pitch_joint"] = {"lower": -0.15, "upper": 0.45}
    control = {"use_sim_handles": False, "control_frequecy": 1000, "joint_list": names,
               "joint_offset": {name: 0.02 if name == "left_shoulder_roll_joint" else 0.0 for name in names},
               "joint_limits": limits, "robot_states": {"stand": {"controllers": ["pd_zero", "pd_stand"]}},
               "controllers": {"pd_zero": zero, "pd_stand": stand, "pd_idle": idle, "rl_unused": {}},
               "sub_joint_state_name": "/joint_states", "pub_joint_cmd_name": "/joint_cmd"}
    dcu = {"joint_list": list(reversed(names)), "transmission": []}
    for name in upper:
        if name.startswith("lumbar_") and name != "lumbar_yaw_joint" or "wrist" in name:
            continue
        dcu["transmission"].append({"type": "SimpleTransmission", "name": name + "_trans", "joint": name,
                                    "actuator": name.replace("joint", "actuator"),
                                    "direction": -1.0 if name == "left_shoulder_roll_joint" else 1.0})
    dcu["transmission"].append({"name": "lumbar_parallel_trans", "type": "LumbarParallelTransmission",
        "joint_pitch": "lumbar_pitch_joint", "joint_roll": "lumbar_roll_joint",
        "actuator_right": "lumbar_right_actuator", "actuator_left": "lumbar_left_actuator",
        "direction_right": -1.0, "direction_left": 1.0})
    for side in ("left", "right"):
        dcu["transmission"].append({"name": side + "_wrist_parallel_trans",
            "type": side.capitalize() + "WristParallelTransmission", "joint_pitch": side + "_wrist_pitch_joint",
            "joint_roll": side + "_wrist_roll_joint", "actuator_right": side + "_wrist_back_actuator",
            "actuator_left": side + "_wrist_front_actuator", "direction_right": 1.0, "direction_left": 1.0})
    motors = [name.replace("_joint", "_actuator") for name in upper if "wrist" not in name and name != "lumbar_pitch_joint" and name != "lumbar_roll_joint"]
    motors += ["lumbar_left_actuator", "lumbar_right_actuator"]
    motors += [f"{side}_wrist_{end}_actuator" for side in ("left", "right") for end in ("front", "back")]
    body = {"name": "body", "ecat_id": 1, "enable": True, "imu_enable": False,
            "channel_1": [], "channel_2": [], "channel_3": []}
    for side, channel in (("left", 1), ("right", 2)):
        arm_names = [f"{side}_{kind}_actuator" for kind in
                     ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_yaw")]
        arm_names += [f"{side}_wrist_{end}_actuator" for end in (("front", "back") if side == "left" else ("back", "front"))]
        body[f"channel_{channel}"] = [{"name": name, "can_id": i + 1,
            "type": "POWER_FLOW_L28" if "wrist" in name else "POWER_FLOW_R86" if i < 2 else "POWER_FLOW_R52"}
            for i, name in enumerate(arm_names)]
    body["channel_3"] = [{"name": f"lumbar_{side}_actuator", "can_id": i + 1, "type": "POWER_FLOW_R86"}
                         for i, side in enumerate(("left", "right"))]
    hip = {"name": "hip", "ecat_id": 2, "enable": True, "imu_enable": True,
           "channel_3": [{"name": "lumbar_yaw_actuator", "can_id": 1, "type": "POWER_FLOW_R86"}]}
    dcu.update({"actuator_list": motors, "enable_actuator": True, "actuator_debug": True,
                "dcu_network": [body, hip]})
    main = {"aimrt": {"plugin": {"plugins": [{"name": "ros2_plugin", "options": {"node_name": "normal"}}]},
        "log": {"backends": [{"type": "rotate_file", "options": {"filename": "normal.log"}}]},
        "channel": {"sub_topics_options": [{"topic_name": ".*", "enable_backends": ["local"]}],
                    "pub_topics_options": [{"topic_name": ".*", "enable_backends": ["local"]}]},
        "module": {"pkgs": [{"enable_modules": ["ControlModule", "DcuDriverModule", "JoyStickModule"]}],
                   "modules": [{"name": "ControlModule", "cfg_file_path": "live_control.yaml"},
                               {"name": "DcuDriverModule", "cfg_file_path": "live_dcu.yaml"},
                               {"name": "JoyStickModule"}]}}}
    return control, main, dcu


class ConfigTests(unittest.TestCase):
    def test_single_joint_modes_hold_lower_body_and_do_not_mutate_live_configs(self):
        source = inputs()
        before = copy.deepcopy(source)
        control, main, plan = tool.build_configs(*source)
        self.assertEqual(source, before)
        self.assertEqual(len(plan["cases"]), 17)
        self.assertEqual(len(plan["lower_joints"]), 12)
        self.assertEqual(plan["stiffness"]["left_shoulder_pitch_joint"], 55.0)
        self.assertEqual(plan["damping"]["left_shoulder_pitch_joint"], 2.0)
        names = plan["joint_names"]
        base = plan["neutral"]
        for case in plan["cases"]:
            cfg = control["controllers"]["pd_" + case["state"]]
            target = dict(zip(names, cfg["init_state"]))
            changed = [name for name in names if target[name] != base[name]]
            self.assertEqual(changed, [case["joint"]])
            self.assertTrue(all(target[n] == base[n] for n in plan["lower_joints"]))
            limits = source[0]["joint_limits"][case["joint"]]
            self.assertTrue(limits["lower"] <= case["target"] <= limits["upper"])
            for field in tool.FIELDS:
                self.assertEqual(len(cfg[field]), 29)
            self.assertNotIn("plan_conf", cfg)
        self.assertEqual({m["name"] for m in main["aimrt"]["module"]["modules"]}, {"ControlModule", "DcuDriverModule"})
        self.assertEqual(main["aimrt"]["module"]["modules"][1]["cfg_file_path"], "live_dcu.yaml")
        self.assertFalse(any(name.startswith("rl_") for name in control["controllers"]))
        for case in plan["cases"]:
            self.assertEqual(control["robot_states"][case["state"]]["pre_states"], ["neutral"])
            self.assertIn(case["state"], control["robot_states"]["neutral"]["pre_states"])
        topics = [state["trigger_topic"] for state in control["robot_states"].values()]
        self.assertTrue(all(topic.startswith(tool.PREFIX) for topic in topics))
        self.assertNotIn("/amp_mode", topics)
        for topic in topics:
            option = main["aimrt"]["channel"]["sub_topics_options"][0]
            self.assertTrue(re.fullmatch(option["topic_name"], topic))
            self.assertIn("ros2", option["enable_backends"])

    def test_reference_axes_and_native_pitch_roll_binding(self):
        _, _, plan = tool.build_configs(*inputs())
        cases = {case["joint"]: case for case in plan["cases"]}
        self.assertAlmostEqual(cases["lumbar_pitch_joint"]["positive_axis"][1], 1.0, places=4)
        self.assertAlmostEqual(cases["lumbar_roll_joint"]["positive_axis"][0], -1.0, places=4)
        self.assertEqual(cases["lumbar_pitch_joint"]["native_target"], -0.10)
        for side in ("left", "right"):
            self.assertEqual(cases[side + "_wrist_pitch_joint"]["actuator"], side + "_wrist_back_actuator")
            self.assertEqual(cases[side + "_wrist_roll_joint"]["actuator"], side + "_wrist_front_actuator")
            self.assertEqual(cases[side + "_shoulder_roll_joint"]["delta"], -0.10)
        for name, case in cases.items():
            self.assertEqual(case["dcu"], "hip" if name == "lumbar_yaw_joint" else "body")
            self.assertEqual(case["channel"], 2 if name.startswith("right") else 1 if name.startswith("left") else 3)
        self.assertEqual(cases["right_wrist_pitch_joint"]["can_id"], 6)
        self.assertEqual(cases["left_wrist_pitch_joint"]["can_id"], 7)

    def test_missing_disabled_and_aliasing_motor_routes_are_rejected(self):
        for fault in ("disabled", "debug_off", "missing_right", "alias", "missing_motor", "bad_id"):
            source = inputs()
            dcu = source[2]
            if fault == "disabled": dcu["enable_actuator"] = False
            if fault == "debug_off": dcu["actuator_debug"] = False
            if fault == "missing_right": dcu["dcu_network"][0]["channel_2"] = []
            if fault == "alias": dcu["dcu_network"][0]["channel_2"][1]["can_id"] = 1
            if fault == "missing_motor": dcu["actuator_list"].remove("right_shoulder_pitch_actuator")
            if fault == "bad_id": dcu["dcu_network"][0]["channel_2"][0]["can_id"] = 9
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                tool.build_configs(*source)

    def test_invalid_or_out_of_range_plans_are_rejected(self):
        for angle in (0, -0.1, 0.21, float("nan")):
            with self.subTest(angle=angle), self.assertRaises(ValueError):
                tool.build_configs(*inputs(), angle=angle)
        source = inputs()
        source[0]["joint_limits"]["left_wrist_pitch_joint"]["upper"] = 0.05
        with self.assertRaisesRegex(ValueError, "测试目标超限"):
            tool.build_configs(*source)
        source = inputs()
        source[0]["controllers"]["pd_stand"]["stiffness"][0] = 700
        with self.assertRaisesRegex(ValueError, "MIT"):
            tool.build_configs(*source)

    def test_feedback_uses_names_and_policy_offsets(self):
        _, _, plan = tool.build_configs(*inputs())
        names = list(reversed(plan["joint_names"]))
        msg = types.SimpleNamespace(name=names, position=[plan["neutral"][n] + plan["offsets"][n] for n in names])
        positions = tool.read_positions(msg, plan["joint_names"])
        self.assertTrue(tool.command_matches(positions, plan["neutral"], plan["offsets"]))
        msg.name[0] = msg.name[1]
        with self.assertRaisesRegex(ValueError, "重复"):
            tool.read_positions(msg, plan["joint_names"])
        with self.assertRaises(ValueError):
            tool.choose_cases(plan, ["left_hip_pitch_joint"])


class ClientTests(unittest.TestCase):
    def exercise(self, answers, fault_at=None, publishers=1, joint="left_shoulder_roll_joint", motor_fault=None, diagnose=False):
        control, main, plan = tool.build_configs(*inputs())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan["bin_dir"] = directory
            plan["sources"] = {}
            for relative, value, key in [(tool.CONTROL_REL, control, "control_sha256"), (tool.MAIN_REL, main, "main_sha256")]:
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                encoded = json.dumps(value)
                path.write_text(encoded)
                plan[key] = hashlib.sha256(encoded.encode()).hexdigest()
            manifest = root / tool.MANIFEST_REL
            manifest.write_text(json.dumps(plan))
            clock = [0.0]
            published = []
            class FakeNode:
                def __init__(self, name):
                    self.callbacks = {}
                    self.timers = []
                    self.state = "idle"
                    self.positions = {n: 0.0 for n in plan["joint_names"]}
                    self.gains = {n: [0.0, 0.0] for n in plan["joint_names"]}
                    self.destroyed = False
                def create_subscription(self, type_, topic, callback, qos):
                    self.callbacks[topic] = callback
                def create_publisher(self, type_, topic, depth):
                    self.assert_mode(topic)
                    def publish(msg):
                        published.append(topic)
                        for state, cfg in control["robot_states"].items():
                            if cfg["trigger_topic"] == topic and self.state in cfg["pre_states"]:
                                self.state = state
                                controller = control["controllers"][cfg["controllers"][0]]
                                if not controller.get("is_keep_controller"):
                                    self.positions = dict(zip(controller["joint_list"], controller["init_state"]))
                                    self.gains = dict(zip(controller["joint_list"], zip(controller["stiffness"], controller["damping"])))
                    return types.SimpleNamespace(publish=publish)
                def assert_mode(self, topic):
                    if not topic.startswith(tool.PREFIX):
                        raise AssertionError("client must not publish joint commands or walking topics")
                def create_timer(self, period, callback):
                    self.timers.append(callback)
                def count_publishers(self, topic): return publishers
                def count_subscribers(self, topic): return 1
                def destroy_node(self): self.destroyed = True
            def spin_once(node, timeout_sec):
                clock[0] += 0.1
                names = list(reversed(plan["joint_names"]))
                msg = types.SimpleNamespace(name=names, position=[node.positions[n] + plan["offsets"][n] for n in names],
                    stiffness=[node.gains[n][0] for n in names], damping=[node.gains[n][1] for n in names])
                node.callbacks[plan["command_topic"]](msg)
                feedback = types.SimpleNamespace(name=names, position=[0.0 if motor_fault == "immobile" and n == joint
                    else node.positions[n] + plan["offsets"][n] for n in names])
                if fault_at is None or clock[0] < fault_at:
                    node.callbacks[plan["feedback_topic"]](feedback)
                bindings = {case["actuator"]: case for case in plan["cases"]}
                motors = list(reversed(plan["actuator_names"]))
                native = {motor: bindings[motor]["direction"] * (node.positions[bindings[motor]["joint"]]
                           + plan["offsets"][bindings[motor]["joint"]]) for motor in motors}
                if motor_fault == "mapping" and node.state.startswith("test_"):
                    native[plan["cases"][0]["actuator"]] += 0.03
                native_feedback = dict(native)
                if motor_fault == "immobile":
                    motor = next(case["actuator"] for case in plan["cases"] if case["joint"] == joint)
                    native_feedback[motor] = 0.0
                native_gains = {motor: list(node.gains[bindings[motor]["joint"]]) for motor in motors}
                if motor_fault == "gain" and node.state.startswith("test_"):
                    native_gains[plan["cases"][0]["actuator"]] = [0.0, 0.0]
                node.callbacks[plan["actuator_command_topic"]](types.SimpleNamespace(name=motors,
                    position=[native[m] for m in motors], stiffness=[native_gains[m][0] for m in motors],
                    damping=[native_gains[m][1] for m in motors]))
                node.callbacks[plan["actuator_feedback_topic"]](types.SimpleNamespace(name=motors,
                    position=[native_feedback[m] for m in motors]))
                for callback in node.timers:
                    callback()
            packages = {name: types.ModuleType(name) for name in ["rclpy", "rclpy.node", "rclpy.qos", "rclpy.signals",
                "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "my_ros2_proto", "my_ros2_proto.msg"]}
            packages["rclpy"].init = lambda **kwargs: None
            packages["rclpy"].shutdown = lambda: None
            packages["rclpy"].spin_once = spin_once
            packages["rclpy.node"].Node = FakeNode
            packages["rclpy.qos"].qos_profile_sensor_data = object()
            packages["rclpy.signals"].SignalHandlerOptions = types.SimpleNamespace(NO=0)
            packages["sensor_msgs.msg"].JointState = type("JointState", (), {})
            packages["my_ros2_proto.msg"].JointCommand = type("JointCommand", (), {})
            packages["std_msgs.msg"].Float32 = type("Float32", (), {})
            responses = iter(answers)
            def readline():
                response = next(responses)
                if isinstance(response, BaseException):
                    raise response
                return response + "\n"
            stdin = types.SimpleNamespace(isatty=lambda: True, readline=readline)
            args = types.SimpleNamespace(manifest=manifest, joint=[joint], seconds=1.0)
            failure = None
            with patch.dict("sys.modules", packages), patch.object(tool, "time", types.SimpleNamespace(monotonic=lambda: clock[0])), \
                    patch.object(tool.sys, "stdin", stdin), patch.object(tool.select, "select", lambda *a: ([stdin], [], [])), \
                    contextlib.redirect_stdout(io.StringIO()):
                try:
                    tool.diagnose(args) if diagnose else tool.run(args)
                except RuntimeError as error:
                    failure = str(error)
            report = json.loads(next((root / "test_logs/data_csv").glob("*.json")).read_text())
            csv_files = list((root / "test_logs/data_csv").glob("*.csv"))
            rows = csv_files[0].read_text() if csv_files else ""
            return published, report, rows, failure

    def test_client_runs_one_case_returns_neutral_and_records_operator_verdict(self):
        published, report, rows, failure = self.exercise(["", "", "r"])
        self.assertIsNone(failure)
        self.assertTrue(report["completed"])
        self.assertEqual(report["results"][0]["observation"], "opposite")
        self.assertEqual(report["results"][0]["tracking_mae"], 0.0)
        self.assertEqual(published[0], tool.PREFIX + "neutral")
        self.assertIn(tool.PREFIX + "left_shoulder_roll", published)
        self.assertEqual(published[-1], tool.PREFIX + "neutral")
        self.assertIn("observe", rows)
        self.assertIn("return_neutral", rows)

    def test_stale_feedback_aborts_to_zero_gain_idle_and_does_not_continue(self):
        published, report, _, failure = self.exercise(["", "", "s"], fault_at=6.0)
        self.assertIn("未更新", failure)
        self.assertFalse(report["completed"])
        self.assertEqual(published[-1], tool.PREFIX + "idle")

    def test_operator_interrupt_requests_measured_pose_keep(self):
        published, report, _, failure = self.exercise(["", "", KeyboardInterrupt()])
        self.assertEqual(failure, "operator interrupt")
        self.assertFalse(report["completed"])
        self.assertEqual(published[-1], tool.PREFIX + "keep")

    def test_competing_joint_command_publisher_is_rejected_before_arming(self):
        published, report, _, failure = self.exercise([], publishers=2)
        self.assertIn("一个发布者", failure)
        self.assertEqual(published, [])
        self.assertFalse(report["completed"])

    def test_all_upper_joint_cases_reach_correct_motor_boundaries(self):
        for joint, *_ in tool.JOINTS:
            with self.subTest(joint=joint):
                _, report, rows, failure = self.exercise(["", "", "s"], joint=joint)
                self.assertIsNone(failure)
                self.assertEqual(report["results"][0]["motion_status"], "responding")
                self.assertIn("actuator_target_", rows)

    def test_right_arm_with_commands_but_no_feedback_motion_is_recorded(self):
        _, report, _, failure = self.exercise(["", "", "n"], joint="right_shoulder_pitch_joint", motor_fault="immobile")
        self.assertIsNone(failure)
        result = report["results"][0]
        self.assertEqual(result["motion_status"], "no_motion")
        self.assertEqual(result["observation"], "no_motion")
        self.assertIn("channel_2", result["route"])

    def test_motor_mapping_or_zero_gain_fault_aborts_before_direction_verdict(self):
        for fault in ("mapping", "gain"):
            with self.subTest(fault=fault):
                _, report, _, failure = self.exercise(["", ""], motor_fault=fault)
                self.assertIn("电机", failure)
                self.assertFalse(report["completed"])
                self.assertEqual(report["results"], [])

    def test_readonly_diagnose_never_publishes_and_reports_motor_route(self):
        published, report, rows, failure = self.exercise([], diagnose=True, joint="right_elbow_pitch_joint")
        self.assertIsNone(failure)
        self.assertEqual(published, [])
        self.assertEqual(rows, "")
        self.assertEqual(report["rows"][0]["joint"], "right_elbow_pitch_joint")
        self.assertIn("channel_2", report["rows"][0]["route"])


class LauncherTests(unittest.TestCase):
    def test_run_script_default_and_static_override_with_fake_hardware_commands(self):
        run_script = tool.ROOT / "motion_control/install/linux/bin/run.sh"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            isolated_run = root / "run.sh"
            isolated_run.write_text(run_script.read_text())
            (root / "cfg").mkdir()
            (root / "fakebin").mkdir()
            for cfg in ("x1_cfg.yaml", "static.yaml"):
                (root / "cfg" / cfg).write_text("{}")
            sudo = root / "fakebin/sudo"
            sudo.write_text('#!/bin/bash\nprintf "sudo %s\\n" "$*" >> "$START_TRACE"\n')
            sudo.chmod(0o755)
            aimrt = root / "aimrt_main"
            aimrt.write_text('#!/bin/bash\nprintf "main %s\\n" "$*" >> "$START_TRACE"\n')
            aimrt.chmod(0o755)
            for custom, expected in [(None, "./cfg/x1_cfg.yaml"), ("./cfg/static.yaml", "./cfg/static.yaml")]:
                trace = root / "trace"
                trace.write_text("")
                env = dict(os.environ, PATH=str(root / "fakebin") + os.pathsep + os.environ["PATH"],
                           START_TRACE=str(trace), NECK_SERVO_PORT=str(root / "no-neck-device"))
                env.pop("AIMRT_CFG_FILE", None)
                if custom: env["AIMRT_CFG_FILE"] = custom
                outcome = subprocess.run(["bash", str(isolated_run)], cwd=root, env=env, capture_output=True, text=True)
                self.assertEqual(outcome.returncode, 0, outcome.stdout + outcome.stderr)
                lines = trace.read_text().splitlines()
                self.assertEqual(lines[0], "sudo setcap cap_net_raw,cap_sys_nice=ep ./aimrt_main")
                self.assertEqual(lines[1], "main --cfg_file_path=" + expected)


if __name__ == "__main__":
    unittest.main()
