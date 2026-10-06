"""Momentum budget and timestep-invariant Poisson rate, independent of Isaac."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
try:
    from storm_model import RainImpulses, rain_envelope

    from model import Config, rectangle, skin_setup
finally:
    sys.path.pop(0)


def setup(count=20):
    c = Config(nx=10, ny=6)
    p, f = rectangle(c)
    centers, _, _ = skin_setup(p, c)
    model = RainImpulses(
        p, f, centers, np.zeros((count, 3)), np.zeros(count), c.fixed_length, c.length
    )
    poses = np.zeros((count, 4, 7))
    poses[:, :, :3] = centers
    poses[:, :, 3] = 1
    return model, poses


def test_rain_momentum_matches_sampled_drop_mass_and_gain():
    model, poses = setup()
    summed = 0
    for _ in range(1000):
        force, torque, _ = model.sample(poses, 200, 1 / 480, 1)
        summed -= force[:, :, 2].sum() / 480
        assert np.isfinite(torque).all()
    assert np.isclose(summed, model.total_drops * model.mass_kg * 7, rtol=1e-6)
    assert np.all(model.hits > 0)
    assert abs(model.total_drops - model.expected_drops) < 6 * np.sqrt(
        model.expected_drops
    )
    expected = (
        200
        * 0.001
        / 3600
        * 1000
        * model.areas.sum()
        * len(poses)
        * (1000 / 480)
        / model.mass_kg
    )
    assert np.isclose(model.expected_drops, expected)


def test_zero_rain_and_vertical_leaf_receive_no_impulses():
    model, poses = setup(2)
    f, t, hit = model.sample(poses, 0, 1 / 480)
    assert not np.any(f) and not np.any(t) and len(hit) == 0
    poses[:, :, 3:] = [np.sqrt(0.5), np.sqrt(0.5), 0, 0]
    for _ in range(50):
        f, _, _ = model.sample(poses, 200, 1 / 480)
        assert not np.any(f)


def test_gain_only_scales_impulses_not_hit_counts():
    a, poses = setup()
    b, _ = setup()
    for _ in range(100):
        fa, ta, _ = a.sample(poses, 200, 1 / 480, 1)
        fb, tb, _ = b.sample(poses, 200, 1 / 480, 20)
        np.testing.assert_allclose(fb, fa * 20, rtol=1e-6, atol=1e-8)
        np.testing.assert_allclose(tb, ta * 20, rtol=1e-6, atol=1e-8)
    np.testing.assert_array_equal(a.hits, b.hits)
    assert (
        rain_envelope(-1) == 0
        and rain_envelope(4) == 0.5
        and rain_envelope(8) == 1
        and rain_envelope(19) == 1
        and rain_envelope(20) == 0
    )


def test_held_force_preserves_impulse_at_four_physics_substeps():
    model, poses = setup()
    total = 0
    for _ in range(100):
        force, _, _ = model.sample(poses, 200, 1 / 120, 1)
        for _ in range(4):
            total -= force[:, :, 2].sum() / 480
    assert np.isclose(
        total, model.total_drops * model.mass_kg * model.speed_m_s, rtol=1e-6
    )
