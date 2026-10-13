"""Exercise actual deployed upper-body CAN routes using production C++ encoders."""
import csv
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("upper", ROOT / "scripts/upper_direction_test.py")
upper = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(upper)


def read_yaml(path):
    try:
        import yaml
        return yaml.safe_load(path.read_text())
    except ImportError:
        if not shutil.which("ruby"):
            raise unittest.SkipTest("actual-config audit needs PyYAML or Ruby YAML")
        return json.loads(subprocess.check_output(
            ["ruby", "-ryaml", "-rjson", "-e", "puts JSON.generate(YAML.load(STDIN.read))"],
            input=path.read_text(), text=True))


class DcuRoutingTests(unittest.TestCase):
    def test_actual_upper_routes_encode_and_receive_in_separate_can_slots(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        module = ROOT / "motion_control/module/dcu_driver_module"
        sdk = module / "xyber_controller/xyber_api"
        control = read_yaml(ROOT / "motion_control/module/control_module/cfg/rl_x1.yaml")
        dcu = read_yaml(module / "cfg/dcu_x1.yaml")
        main = read_yaml(ROOT / "motion_control/install/linux/bin/cfg/x1_cfg_real_nav.yaml")
        _, _, plan = upper.build_configs(control, main, dcu)
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            cases = tmp / "cases.csv"
            with cases.open("w") as stream:
                writer = csv.writer(stream, lineterminator="\n")
                for case in plan["cases"]:
                    joint = case["joint"]
                    writer.writerow([case["actuator"], case["ecat_id"], case["channel"], case["can_id"],
                        case["actuator_type"], case["native_neutral"], case["native_target"],
                        plan["stiffness"][joint], plan["damping"][joint]])
            executable = tmp / "dcu_routing"
            compiled = subprocess.run([compiler, "-std=c++20", "-fsanitize=undefined",
                "-fno-sanitize-recover=all", "-I", str(sdk / "include"),
                str(ROOT / "scripts/tests/test_dcu_routing.cpp"), str(sdk / "src/dcu.cpp"),
                str(sdk / "src/power_flow.cpp"), "-o", str(executable)], capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            ran = subprocess.run([str(executable), str(cases)], capture_output=True, text=True)
            self.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)
            self.assertNotIn("runtime error", ran.stderr)
            self.assertEqual(ran.stdout.count("PASS "), 17)
            for case in plan["cases"]:
                self.assertIn(f"PASS {case['actuator']} ECAT {case['ecat_id']} "
                              f"channel {case['channel']} CAN {case['can_id']}", ran.stdout)


if __name__ == "__main__":
    unittest.main()
