"""Reproducible staggered rain, without Isaac imports."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
try:
    from rain_scene import rain_schedule
finally:
    sys.path.pop(0)


def test_staggered_seeded_releases_above_whole_canopy():
    points = np.array([[0.04, -0.01, 0.15], [0.06, 0.01, 0.15]])
    offsets = np.array([[0, 0, 0.1], [0.1, 0, 0.2], [0, 0.1, 0.3]])
    yaws = np.array([0, 1, 2])
    result = rain_schedule(offsets, yaws, points, 0.15, 0.02, 24, 42)
    assert result == rain_schedule(offsets, yaws, points, 0.15, 0.02, 24, 42)
    assert result != rain_schedule(offsets, yaws, points, 0.15, 0.02, 24, 43)
    dt = np.diff([r["after_start_s"] for r in result])
    assert np.all((dt >= 0.6) & (dt <= 1.4))
    assert all(0.57 <= r["position_m"][2] <= 0.65 for r in result)
    assert {r["target_leaf"] for r in result} == {0, 1, 2}


def test_release_writes_only_selected_body_with_full_view_buffers(monkeypatch):
    import types

    from rain_scene import TomatoRain

    class Attr:
        def Set(self, value):
            pass

    class View:
        def __init__(self):
            self.transforms = np.arange(21, dtype=np.float32).reshape(3, 7)
            self.velocities = np.ones((3, 6), dtype=np.float32)
            self.writes = []

        def get_transforms(self):
            return self.transforms

        def get_velocities(self):
            return self.velocities

        def set_transforms(self, values, indices):
            assert values.shape == (3, 7)
            self.writes.append(indices.tolist())
            self.transforms[indices] = values[indices]

        def set_velocities(self, values, indices):
            assert values.shape == (3, 6)
            self.velocities[indices] = values[indices]

    monkeypatch.setitem(
        sys.modules,
        "pxr",
        types.SimpleNamespace(
            UsdGeom=types.SimpleNamespace(
                Imageable=lambda _: types.SimpleNamespace(MakeVisible=lambda: None)
            )
        ),
    )
    rain = object.__new__(TomatoRain)
    rain.view = View()
    old = rain.view.transforms.copy()
    rain.indices = np.array([2, 0, 1], dtype=np.int32)
    rain.schedule = [{"position_m": [0, 0, 0.7]}] * 3
    rain.collisions = rain.gravity = [Attr()] * 3
    rain.visuals = ["v"] * 3
    rain.stage = types.SimpleNamespace(GetPrimAtPath=lambda _: None)
    rain.release(1)
    assert rain.view.writes == [[0]]
    np.testing.assert_array_equal(rain.view.transforms[1:], old[1:])
    np.testing.assert_array_equal(rain.view.velocities[1:], np.ones((2, 6)))


def test_progressive_waves_are_simultaneous_and_start_without_overlap():
    points = np.array([[0.04, -0.01, 0.15], [0.06, 0.01, 0.15]])
    offsets = np.array([[i * 0.09, j * 0.09, 0] for i in range(4) for j in range(4)])
    waves = [1, 1, 2, 3, 4, 6, 8, 10, 12]
    result = rain_schedule(
        offsets, np.zeros(16), points, 0.15, 0.02, sum(waves), 42, waves
    )
    assert len(result) == 47
    previous = None
    for wave, size in enumerate(waves):
        rows = [r for r in result if r["wave"] == wave]
        assert len(rows) == size
        times = {r["after_start_s"] for r in rows}
        assert len(times) == 1
        current = next(iter(times))
        if previous is not None:
            assert 2 <= current - previous <= 3.5
        previous = current
        xyz = np.array([r["position_m"] for r in rows])
        for i in range(size):
            assert np.all(np.linalg.norm(xyz[i + 1 :] - xyz[i], axis=1) > 0.045)
