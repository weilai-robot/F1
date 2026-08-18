#!/usr/bin/env python3
"""Extract the SPI result payload from a torch.save'd .pt without torch.

torch.save (new zipfile serialization) stores the pickled object as
``data.pkl`` inside a zip container. Our payload contains only plain
python/numpy types, so a stock ``pickle.load`` works and torch is not needed.

Usage: python extract_pt.py in.pt out.json
"""
import io
import json
import pickle
import sys
import zipfile
from pathlib import Path


def _numpyify(obj):
    """Best-effort conversion to plain-JSON types (arrays -> nested lists)."""
    import numpy as np
    if isinstance(obj, dict):
        return {k: _numpyify(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_numpyify(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


def main() -> None:
    src, dst = sys.argv[1], sys.argv[2]
    with zipfile.ZipFile(src) as z:
        names = z.namelist()
        pkl = next(n for n in names if n.endswith("data.pkl"))
        with z.open(pkl) as f:
            payload = pickle.load(f, encoding="latin1")
    payload = _numpyify(payload)
    Path(dst).write_text(json.dumps(payload, indent=2))
    print("keys:", list(payload.keys()))
    print("best_cost:", payload.get("best_cost"),
          "| nominal_cost:", payload.get("nominal_cost"),
          "| n_clips:", payload.get("n_clips"))


if __name__ == "__main__":
    main()
