"""Compatibility entry point; prefer ``python -m exporterV1``."""

from exporterV1.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
