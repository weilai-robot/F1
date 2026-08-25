#!/usr/bin/env bash
# gradmotion remote bootstrap: install deps + run the full SPI sysid pipeline.
# startScript form:
#   gm-run F1/sim2real/scripts/remote_sysid.sh
#
# Layout on the platform: repos are cloned side by side:
#   ./F1/                (this repo, branch dev/sim2real-spi)
#   ./Humanoid_motion/   (submodule content: real data + MJCF)
# We symlink ./F1/motion_control -> ../Humanoid_motion so the config's
# "motion_control/..." paths resolve.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"          # .../F1/sim2real/scripts
F1_ROOT="$(cd "$HERE/../.." && pwd)"           # .../F1
cd "$F1_ROOT"

# mode: full pipeline (default) or --validate-only (dataset + validation +
# apply; skips the ~15 min CMA-ES identification, reuses uploaded params)
MODE="${1:-full}"
if [ "$MODE" = "--validate-only" ]; then
  echo "remote_sysid: VALIDATE-ONLY mode (skip identification)"
fi

# --- locate sibling Humanoid_motion checkout and link it -------------------
if [ ! -d motion_control/czy ]; then
  for CAND in "../Humanoid_motion" "../../Humanoid_motion" "Humanoid_motion"; do
    if [ -d "$CAND/czy" ]; then
      rm -rf motion_control
      ln -s "$(cd "$CAND" && pwd)" motion_control
      break
    fi
  done
fi
if [ ! -d motion_control/czy ]; then
  # robust fallback: any sibling dir that contains the real-data tree
  for CAND in ../*/ ../../*/; do
    if [ -d "${CAND}czy/real_data" ]; then
      rm -rf motion_control
      ln -s "$(cd "$CAND" && pwd)" motion_control
      break
    fi
  done
fi
if [ ! -d motion_control/czy ]; then
  # last resort: init the git submodule (requires git access)
  git submodule update --init motion_control || true
fi
# 辨识数据源自 2026-08-24 起随仓库分发（sim2real/data/），real_data 目录不再是
# 硬依赖；但 MJCF 模型仍来自 motion_control（或兄弟仓库软链）
MJCF_PATH="motion_control/module/sim_module/model/mjcf/xyber_x1_flat.xml"
if [ ! -f "$MJCF_PATH" ]; then
  echo "FATAL: MJCF not found at $MJCF_PATH" >&2
  exit 3
fi

# --- python deps (image ships torch; mujoco/optuna are pip-only) ----------
python -m pip install -q --no-input mujoco optuna cmaes pyyaml matplotlib 2>&1 | tail -1 || true
python - <<'PY'
import mujoco, optuna, cmaes, yaml, numpy
print("deps OK:", "mujoco", mujoco.__version__, "| optuna", optuna.__version__,
      "| cmaes", cmaes.__version__)
PY

# --- stage 0: dataset ------------------------------------------------------
python sim2real/scripts/prepare_dataset.py \
  --config sim2real/configs/x1_spi.yaml \
  --out sim2real/data/x1_clips.npz

if [ "$MODE" != "--validate-only" ]; then
  # --- stage 1: SPI identification ----------------------------------------
  python sim2real/scripts/run_spi.py \
    --config sim2real/configs/x1_spi.yaml \
    --dataset sim2real/data/x1_clips.npz \
    --out-dir logs/spi_sysid
else
  echo "remote_sysid: [validate-only] params must come from a previous run"
  # fresh container has no logs/ — fall back to the repo-committed params file
  # (reconstructed from the identification task log; same payload format)
  if [ ! -f logs/spi_sysid/gm_play/identified_params.json ]; then
    mkdir -p logs/spi_sysid/gm_play
    cp sim2real/results/v14_identified_params.json \
       logs/spi_sysid/gm_play/identified_params.json
    echo "remote_sysid: [validate-only] restored params from sim2real/results/"
  fi
  ls -la logs/spi_sysid/gm_play/identified_params.json
fi

# --- stage 1.5: validation (completion criteria, 完成标准) ----------------
# exit code: 0=PASS 1=FAIL 2=error; run after artifacts so logs always exist
# (set -e would kill the pipeline on rc=1 before diagnostics run — capture it)
VALIDATE_RC=0
python sim2real/scripts/validate_spi.py \
  --config sim2real/configs/x1_spi.yaml \
  --dataset sim2real/data/x1_clips.npz \
  --params logs/spi_sysid/gm_play/identified_params.json \
  --out-dir logs/spi_sysid || VALIDATE_RC=$?
echo "remote_sysid: validation exit code = $VALIDATE_RC"

# --- diagnostics: mass landscape ------------------------------------------
python sim2real/scripts/mass_landscape.py \
  --config sim2real/configs/x1_spi.yaml \
  --dataset sim2real/data/x1_clips.npz \
  --out-dir logs/mass_landscape || true

# --- apply params (URDF/MJCF patch + DR config) ---------------------------
python sim2real/scripts/apply_params.py \
  --params logs/spi_sysid/gm_play/identified_params.json \
  --out-dir sim2real/export || true

echo "remote_sysid: ALL DONE (validation rc=$VALIDATE_RC)"
ls -R logs/ | head -40

# propagate the validation verdict as the task exit code: 0 PASS / 1 FAIL
exit "$VALIDATE_RC"
