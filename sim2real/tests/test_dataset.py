import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from spi.dataset import (FULL_JOINT_ORDER, LEG_JOINTS, MODE_POS, MODE_TAU,
                         load_clips, parse_csv, save_clips, segment_clips)

REPO = Path(__file__).resolve().parents[2]  # .../F1


def synth_csv(n=300, dt=0.01, parallel_ankles=True):
    t = np.arange(n) * 1e9 * dt  # ns
    header = ["timestamp_ns", "phase_sin", "phase_cos", "cmd_linear_x",
              "cmd_linear_y", "cmd_angular_z", "left_contact", "right_contact",
              "base_euler_x", "base_euler_y", "base_euler_z",
              "base_ang_vel_x", "base_ang_vel_y", "base_ang_vel_z"]
    rows = []
    for i in range(n):
        rows.append([int(t[i]), 0.0, 1.0, 0.2, 0.0, 0.0, 1, 1,
                     0.01, 0.02, 1.77, 0.001, -0.002, 0.003])
    for jn in FULL_JOINT_ORDER[:17]:
        header += [f"action_{jn}", f"pos_{jn}", f"vel_{jn}", f"effort_{jn}",
                   f"pos_des_raw_{jn}", f"pos_des_lpf_{jn}",
                   f"tau_des_raw_{jn}", f"tau_des_lpf_{jn}", f"is_parallel_{jn}"]
        for i in range(n):
            rows[i] += [0.1, np.sin(i * 0.05) * 0.3, 0.5 * np.cos(i * 0.05),
                        10.0 * np.sin(i * 0.05), 0.31, 0.31, "nan", "nan",
                        0 if parallel_ankles else 0]
    for jn in LEG_JOINTS:
        header += [f"action_{jn}", f"pos_{jn}", f"vel_{jn}", f"effort_{jn}",
                   f"pos_des_raw_{jn}", f"pos_des_lpf_{jn}",
                   f"tau_des_raw_{jn}", f"tau_des_lpf_{jn}", f"is_parallel_{jn}"]
        par = 1 if (parallel_ankles and "ankle" in jn) else 0
        for i in range(n):
            q = np.sin(i * 0.05 + hash(jn) % 10) * 0.2
            rows[i] += [0.1, q, 0.1 * np.cos(i * 0.05), 5.0 * np.sin(i * 0.05),
                        q + 0.05, q + 0.04,
                        (2.0 if par else "nan"), (1.9 if par else "nan"), par]
    header += ["clip_count", "imu_quat_w", "imu_quat_x", "imu_quat_y", "imu_quat_z",
               "imu_gyro_x", "imu_gyro_y", "imu_gyro_z",
               "imu_accel_x", "imu_accel_y", "imu_accel_z"]
    for i in range(n):
        rows[i] += [0, 0.63, 0.0, 0.0, 0.77, 0.01, -0.02, 0.03, 0.1, -0.1, 9.8]
    return header, rows


class TestDataset(unittest.TestCase):
    def test_parse_and_segment(self):
        header, rows = synth_csv()
        tmp = Path(__file__).parent / "_synth.csv"
        with open(tmp, "w") as f:
            f.write(",".join(header) + "\n")
            for r in rows:
                f.write(",".join(str(x) for x in r) + "\n")
        try:
            log = parse_csv(tmp, kp={"left_hip_pitch_joint": 40.0},
                            kd={"left_hip_pitch_joint": 3.0})
            self.assertEqual(log.q.shape, (300, 29))
            self.assertAlmostEqual(log.dt, 0.01, places=5)
            # ankle parallel -> MODE_TAU, hip -> MODE_POS
            i_ankle = FULL_JOINT_ORDER.index("left_ankle_pitch_joint")
            i_hip = FULL_JOINT_ORDER.index("left_hip_pitch_joint")
            self.assertEqual(log.mode[0, i_ankle], MODE_TAU)
            self.assertEqual(log.mode[0, i_hip], MODE_POS)
            self.assertFalse(np.isnan(log.target_tau[0, i_ankle]))
            self.assertTrue(np.isnan(log.target_tau[0, i_hip]))
            # kp applied
            self.assertEqual(log.kp[i_hip], 40.0)
            self.assertEqual(log.kd[i_hip], 3.0)
            clips = segment_clips(log, h_min_s=0.5, h_max_s=1.0, seed=0)
            self.assertGreater(len(clips), 1)
            for c in clips:
                self.assertGreaterEqual(c["n"], 5)
                self.assertLessEqual(c["n"] * log.dt, 1.0 + 2 * log.dt)
            # roundtrip through npz
            out = Path(__file__).parent / "_synth_clips.npz"
            save_clips(clips, {"src": "synth"}, out)
            clips2, meta = load_clips(out)
            self.assertEqual(len(clips2), len(clips))
            self.assertEqual(meta["src"], "synth")
            c0, c0b = clips[0], clips2[0]
            np.testing.assert_allclose(c0["q0"], c0b["q0"])
            np.testing.assert_allclose(c0["ref_q"], c0b["ref_q"])
            out.unlink()
        finally:
            tmp.unlink()

    def test_real_csv_if_present(self):
        real = REPO / "motion_control/czy/real_data/round_exp_010_kp40_kd3_ff0_20260804_105929.csv"
        if not real.exists():
            self.skipTest("real data not checked out")
        log = parse_csv(real, kp={j: 40.0 for j in LEG_JOINTS},
                        kd={j: 3.0 for j in LEG_JOINTS})
        self.assertEqual(log.q.shape[1], 29)
        self.assertTrue(np.isfinite(log.q[:, 17:]).all())
        self.assertAlmostEqual(log.dt * 1e3, 9.6, delta=1.0)
        clips = segment_clips(log, h_min_s=1.0, h_max_s=2.0)
        self.assertGreater(len(clips), 5)


if __name__ == "__main__":
    unittest.main()
