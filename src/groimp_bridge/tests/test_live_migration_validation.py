"""Opt-in real-plant acceptance tests for migration Phases C and D."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import pytest

from groimp_bridge.client import GroIMPConnectionError
from groimp_bridge.migration_validation import validate_project


pytestmark = pytest.mark.groimp
PROJECT_ROOT = Path(__file__).resolve().parents[3]
PROJECT = PROJECT_ROOT / "model" / "project_bridge.gsz"


def _require_live(slow=False):
    if os.environ.get("RUN_GROIMP_TESTS") != "1":
        pytest.skip("set RUN_GROIMP_TESTS=1 to enable GroIMP integration tests")
    if slow and os.environ.get("RUN_GROIMP_SLOW_TESTS") != "1":
        pytest.skip("set RUN_GROIMP_SLOW_TESTS=1 for mature-plant validation")


def _hashes():
    paths = [PROJECT]
    for directory in (PROJECT_ROOT / "model" / "input", PROJECT_ROOT / "model" / "output"):
        paths.extend(path for path in directory.rglob("*") if path.is_file())
    return {
        str(path.relative_to(PROJECT_ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(paths)
    }


def _validate_or_skip(day):
    try:
        return validate_project(PROJECT, steps=day)
    except GroIMPConnectionError as exc:
        pytest.skip(f"GroIMP server is unavailable: {exc}")


def test_day_1_geometry_and_all_representations_pass_without_source_changes():
    _require_live()
    before = _hashes()
    bundle = _validate_or_skip(1)
    assert _hashes() == before
    assert len(bundle.inspection.snapshot.nodes) == 51
    assert len(bundle.inspection.snapshot.edges) == 50
    assert bundle.geometry_validation.summary["failed"] == 0
    assert bundle.geometry_validation.summary["not_recoverable"] == 0
    assert bundle.comparison.metadata["status"] == "passed"
    assert bundle.comparison.counts["native_biological"]["Internode"] == 3
    assert bundle.comparison.counts["native_biological"]["Leaf"] == 5


@pytest.mark.slow
@pytest.mark.parametrize("day", [25, 80])
def test_mature_days_cover_branches_leaves_trusses_pedicels_and_fruits(day):
    _require_live(slow=True)
    before = _hashes()
    bundle = _validate_or_skip(day)
    assert _hashes() == before
    roles = set(bundle.geometry.diagnostics["covered_roles"])
    assert {"internode", "petiole", "leaf_rachis", "truss_rachis", "pedicel", "fruit"} <= roles
    assert bundle.geometry_validation.summary["failed"] == 0
    assert bundle.geometry_validation.summary["not_recoverable"] == 0
    assert bundle.comparison.metadata["status"] == "passed"
    assert bundle.comparison.diagnostics["classifications"]["UNKNOWN_DIFFERENCE"] == 0
    assert bundle.comparison.diagnostics["classifications"]["LIKELY_BUG"] == 0
