"""Filesystem isolation for GroIMP projects whose RGG functions write outputs."""

from __future__ import annotations

from contextlib import contextmanager
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Iterator
from zipfile import ZipFile


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


def _runtime_string(path: Path) -> str:
    return str(path.resolve()).replace("\\", "\\\\").replace('"', '\\"') + "/"


def override_runtime_paths(source: str, runtime_root: Path) -> str:
    """Replace this model's getWD-based external paths in RGG source text."""

    substitutions = {
        "PATH_INPUT": _runtime_string(runtime_root / "input"),
        "PATH_OUTPUT": _runtime_string(runtime_root / "output"),
    }
    for name, value in substitutions.items():
        pattern = re.compile(
            rf"(^\s*static\s+String\s+{name}\s*=\s*)"
            rf'getWD\(\)\+"(?:input|output)/"(\s*;)',
            re.MULTILINE,
        )
        source, count = pattern.subn(rf'\1"{value}"\2', source, count=1)
        if count != 1:
            raise ValueError(f"Cannot isolate {name}: declaration not found")
    return source


def configure_isolated_paths(project_path: Path, runtime_root: Path) -> None:
    """Point one disposable GSZ at its own external input/output folders.

    GroIMP 2.2.1's headless ``getWD()`` can resolve to the account home rather
    than the opened GSZ directory. The model initializes file-backed static
    fields while the project is opening, before the API can update or compile
    a source file. Therefore the path override must be applied to the copied
    archive itself before ``openWB``.
    """

    entry_name = "param/parameters.rgg"
    replacement = project_path.with_suffix(".rewritten.gsz")
    found = False
    with ZipFile(project_path, "r") as source, ZipFile(replacement, "w") as target:
        for info in source.infolist():
            payload = source.read(info.filename)
            if info.filename == entry_name:
                found = True
                text = payload.decode("windows-1252")
                payload = override_runtime_paths(text, runtime_root).encode(
                    "windows-1252"
                )
            target.writestr(info, payload)
    if not found:
        replacement.unlink(missing_ok=True)
        raise ValueError(f"Cannot isolate GroIMP project: missing {entry_name}")
    os.replace(replacement, project_path)


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
        configure_isolated_paths(runtime_project, runtime_root)
        yield runtime_project
