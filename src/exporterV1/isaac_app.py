"""Open an already generated static V1 stage in Isaac Sim."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Callable


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--smoke-frames", type=int, default=10)
    args, kit_args = parser.parse_known_args()
    # ``--usd`` is also a native Kit flag. If it reaches SimulationApp, Kit
    # opens the stage during bootstrap and this loader opens it a second time;
    # in GUI mode that duplicate open can immediately end the app loop.
    sys.argv = [sys.argv[0], *kit_args]
    return args


def _open_stage_and_wait(
    context,
    app,
    usd_path: Path,
    is_stage_loading: Callable[[], bool],
) -> Path | None:
    """Open a stage using the Isaac 4.5 API and return its resolved root path."""

    # Isaac Sim 4.5 opens the stage successfully but returns ``None`` from
    # open_stage(), so its return value must not be treated as a boolean.
    context.open_stage(str(usd_path))
    while is_stage_loading():
        app.update()
    # Process the stage-open/window events before querying the new root layer.
    app.update()
    opened_stage = context.get_stage()
    if opened_stage is None or not opened_stage.GetRootLayer().realPath:
        return None
    return Path(opened_stage.GetRootLayer().realPath).resolve()


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
        from isaacsim.core.utils.stage import is_stage_loading

        context = omni.usd.get_context()
        opened_path = _open_stage_and_wait(context, app, usd_path, is_stage_loading)
        if opened_path != usd_path:
            print(
                f"[ERROR] Isaac Sim opened {opened_path!s} instead of {usd_path}",
                file=sys.stderr,
            )
            return 1
        print(f"[OK] Isaac Sim opened static V1 stage: {usd_path}", flush=True)
        if args.headless:
            for _ in range(max(args.smoke_frames, 1)):
                app.update()
        else:
            # SimulationApp.is_running() also requires get_stage() to be
            # non-null. Isaac Sim 4.5 briefly clears it while replacing the
            # bootstrap stage, which used to terminate this GUI loop. Follow
            # the actual Kit window lifecycle instead.
            while app.app.is_running() and not app.is_exiting():
                app.update()
        return 0
    except KeyboardInterrupt:
        print("[INFO] Isaac Sim interrupted by user.", flush=True)
        return 0
    except Exception as exc:
        import traceback

        print(f"[ERROR] Isaac Sim V1 loader failed: {exc}", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
