"""SPI core: sampling-based parameter identification for legged sim2real.

Reference: "Sampling-Based System Identification with Active Exploration for
Legged Robot Sim2Real Learning" (SPI-Active), arXiv:2505.14266, CoRL 2025.
"""

from .param_space import (PHI_NAMES, BodyParams, MotorGroup, ParamSpace,
                          phi_search_box, phi_to_physical, phi_to_U,
                          physical_to_phi, tanh_motor_torque)
from .dataset import (FULL_JOINT_ORDER, JIDX, LEG_JOINTS, PARALLEL_JOINTS,
                      RobotLog, load_clips, parse_csv, save_clips, segment_clips)
from .cost import CostWeights, PredictionCost, quat_err, total_cost
from .optimizer import build_space, run_spi

__all__ = [
    "PHI_NAMES", "BodyParams", "MotorGroup", "ParamSpace", "phi_search_box",
    "phi_to_physical", "phi_to_U", "physical_to_phi", "tanh_motor_torque",
    "FULL_JOINT_ORDER", "JIDX", "LEG_JOINTS", "PARALLEL_JOINTS", "RobotLog",
    "load_clips", "parse_csv", "save_clips", "segment_clips",
    "CostWeights", "PredictionCost", "quat_err", "total_cost",
    "build_space", "run_spi",
]
