from __future__ import annotations

import json

from .config import SimConfig
from .simulation import run_experiments


def main() -> None:
    cfg = SimConfig()
    summary = run_experiments(cfg, outputs_dir="outputs")
    print(json.dumps(summary["strategies"], indent=2))
    print("\nWrote plots + outputs/summary.json")


if __name__ == "__main__":
    main()

