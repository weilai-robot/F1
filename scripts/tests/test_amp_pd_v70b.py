#!/usr/bin/env python3
"""Guard: rl_x1.yaml / rl_x1_sim.yaml rl_walk_amp PD + policy must match the
v68 sim2real identification (x1_armature_pd.md §4, 2026-10-14 方案 B) and the
v70b m16000b policy artifact.

Ground truth arrays are embedded from x1_armature_pd.md §4 (generated/verified
by X1_29_AMP/roboparty_train/check_v68_params.py). Run:
    python3 scripts/tests/test_amp_pd_v70b.py
"""
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

STIFFNESS = [65.0, 90.0, 65.0, 60.0, 70.0, 60.0,
             100.0, 100.0, 100.0, 85.0, 40.0, 40.0,
             85.0, 50.0, 40.0, 40.0, 50.0, 50.0,
             40.0, 40.0, 50.0, 30.0, 30.0, 20.0,
             20.0, 15.0, 15.0, 15.0, 15.0]
DAMPING = [8.0, 8.0, 8.0, 8.0, 8.0, 8.0,
           3.3, 8.0, 3.3, 8.0, 4.0, 4.0,
           8.0, 4.0, 4.0, 4.0, 4.0, 2.5,
           2.0, 2.0, 2.5, 1.9, 1.9, 1.0,
           1.0, 1.0, 1.0, 1.0, 1.0]
POLICY_FILE = "cfg/control_module/policy/x1_amp_v70b_m16000_50hz.onnx"
POLICY_ORDER = [
    "left_hip_pitch_joint", "lumbar_yaw_joint", "right_hip_pitch_joint",
    "left_hip_roll_joint", "lumbar_roll_joint", "right_hip_roll_joint",
    "left_hip_yaw_joint", "lumbar_pitch_joint", "right_hip_yaw_joint",
    "left_knee_pitch_joint", "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "right_knee_pitch_joint", "left_ankle_pitch_joint", "left_shoulder_roll_joint",
    "right_shoulder_roll_joint", "right_ankle_pitch_joint", "left_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint", "right_ankle_roll_joint",
    "left_elbow_pitch_joint", "right_elbow_pitch_joint", "left_elbow_yaw_joint",
    "right_elbow_yaw_joint", "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint",
]


def _section(text: str) -> str:
    m = re.search(r"^  rl_walk_amp:\n(.*?)(?=^  \w|\Z)", text, re.M | re.S)
    assert m, "rl_walk_amp section not found"
    return m.group(1)


def _array(section: str, key: str):
    m = re.search(rf"^\s*{key}:\s*\[([^\]]*)\]", section, re.M)
    assert m, f"{key} array not found in rl_walk_amp"
    return [float(x) for x in m.group(1).replace("\n", " ").split(",") if x.strip()]


class AmpPdV70bTests(unittest.TestCase):
    def _yaml(self, name):
        return (ROOT / "motion_control" / "module" / "control_module" / "cfg" / name).read_text()

    def test_pd_and_policy_real(self):
        s = _section(self._yaml("rl_x1.yaml"))
        self.assertEqual(_array(s, "stiffness"), STIFFNESS)
        self.assertEqual(_array(s, "damping"), DAMPING)
        self.assertIn(POLICY_FILE, s)

    def test_pd_and_policy_sim(self):
        s = _section(self._yaml("rl_x1_sim.yaml"))
        self.assertEqual(_array(s, "stiffness"), STIFFNESS)
        self.assertEqual(_array(s, "damping"), DAMPING)
        self.assertIn(POLICY_FILE, s)

    def test_pd_within_mit_kd_range(self):
        # R86/R52 主机 MIT kd 编码上限 8.0（x1_armature_pd.md 约束）
        self.assertLessEqual(max(DAMPING), 8.0 + 1e-9)

    def test_joint_list_matches_policy_order(self):
        s = _section(self._yaml("rl_x1.yaml"))
        m = re.search(r"joint_list:\s*\[([^\]]*)\]", s)
        names = [x.strip() for x in m.group(1).replace("\n", " ").split(",")]
        self.assertEqual(names, POLICY_ORDER)

    def test_policy_artifact_exists(self):
        p = ROOT / "motion_control" / "module" / "control_module" / "policy" / "x1_amp_v70b_m16000_50hz.onnx"
        self.assertTrue(p.exists(), f"missing {p}")

    def test_run_amp_real_script_references_new_model(self):
        sh = (ROOT / "scripts" / "run_amp_real.sh").read_text()
        self.assertIn("x1_amp_v70b_m16000_50hz.onnx", sh)
        self.assertNotIn("x1_amp_soup59c_50hz.onnx", sh)


if __name__ == "__main__":
    sys.exit(unittest.main(verbosity=2)[0])
