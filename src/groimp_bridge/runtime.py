"""Filesystem isolation for GroIMP projects whose RGG functions write outputs."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import shutil
import tempfile
from typing import Iterator


OUTPUT_DIRECTORIES = (
    "dynamic_output/crop",
    "dynamic_output/graphs",
    "dynamic_output/organ",
    "dynamic_output/plant",
    "sensor_output",
    "static_output/crop",
    "static_output/organ",
    "static_output/plant",
)


@contextmanager
def isolated_project(project_path: str | Path) -> Iterator[Path]:
    """Copy a GSZ project and its external inputs to a disposable directory."""

    source = Path(project_path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"GroIMP project not found: {source}")
    if source.suffix.lower() != ".gsz":
        raise ValueError(f"Expected a .gsz GroIMP project, got: {source}")

    input_dir = source.parent / "input"
    if not input_dir.is_dir():
        raise FileNotFoundError(
            f"GroIMP input directory not found beside project: {input_dir}"
        )

    with tempfile.TemporaryDirectory(prefix="autotom-groimp-") as temp_name:
        runtime_root = Path(temp_name)
        runtime_project = runtime_root / source.name
        shutil.copy2(source, runtime_project)
        shutil.copytree(input_dir, runtime_root / "input")
        for relative in OUTPUT_DIRECTORIES:
            (runtime_root / "output" / relative).mkdir(parents=True, exist_ok=True)
        yield runtime_project
