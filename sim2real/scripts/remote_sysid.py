#!/usr/bin/env python3
"""Python entry point for gradmotion.

The platform's ``gm-run`` launcher executes entry files with the image's
python interpreter, so a bare ``.sh`` startScript fails with SyntaxError.
This wrapper simply re-execs ``remote_sysid.sh`` under bash; the script
self-locates via ``$(dirname $0)`` so the cwd does not matter.

startScript form:  gm-run F1/sim2real/scripts/remote_sysid.py [--validate-only]
"""
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SH = HERE / "remote_sysid.sh"

if __name__ == "__main__":
    args = sys.argv[1:]
    sys.exit(subprocess.call(["bash", str(SH)] + args))
