"""Open an already generated static V1 stage in Isaac Sim."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--smoke-frames", type=int, default=10)
    return parser.parse_args()


def main() -> int:
    args = _arguments()
    usd_path = args.usd.expanduser().resolve()
    if not usd_path.is_file():
        print(f"[ERROR] USD stage does not exist: {usd_path}", file=sys.stderr)
        return 2

    # Isaac modules must be imported only after command-line parsing.
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})
    try:
        import omni.usd

        context = omni.usd.get_context()
        if not context.open_stage(str(usd_path)):
            print(f"[ERROR] Isaac Sim could not open: {usd_path}", file=sys.stderr)
            return 1
        while context.is_stage_loading():
            app.update()
        print(f"[OK] Isaac Sim opened static V1 stage: {usd_path}", flush=True)
        if args.headless:
            for _ in range(max(args.smoke_frames, 1)):
                app.update()
        else:
            while app.is_running():
                app.update()
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
