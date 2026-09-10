"""Interaction geometry tests independent of the Isaac runtime."""
import numpy as np
import pytest
import io
import sys
from types import SimpleNamespace

from exporterV2.fruit_interaction import GuiDragBridge, LimitedDrag, pick_ray, unit


def test_occluding_support_is_not_accepted_as_fruit():
    calls = []
    def query(origin, direction, distance):
        calls.append((origin, direction))
        assert np.linalg.norm(direction) == pytest.approx(1)
        if len(calls) == 1:
            return {"hit": True, "rigidBody": "/support", "position": [1, 0, 0]}
        return {"hit": True, "rigidBody": "/fruit", "position": [.01, 0, 0], "collision": "/fruit/shape"}
    eye, direction, hit, attempts = pick_ray([0, 0, 0], "/fruit", query)
    assert len(attempts) == 2
    assert attempts[0]["rigid_body"] == "/support"
    assert attempts[-1]["collider"] == "/fruit/shape"
    assert hit == pytest.approx([.01, 0, 0])


def test_occluded_target_fails_instead_of_grabbing_another_body():
    with pytest.raises(ValueError, match="no unobstructed fruit ray"):
        pick_ray([0, 0, 0], "/fruit", lambda *args: {"hit": True, "rigidBody": "/support"})


@pytest.mark.parametrize("vector", [[0, 0, 0], [np.nan, 0, 0], [np.inf, 0, 0]])
def test_invalid_ray_is_rejected(vector):
    with pytest.raises(ValueError, match="invalid interaction ray"):
        unit(vector)


def test_rapid_drag_and_direction_reversal_obey_vector_force_limits():
    drag = LimitedDrag()
    drag.begin([0, 0, 0], [0, 0, 0], [1, 0, 0])
    previous = np.zeros(3)
    for step in range(1200):
        side = 1 if step < 450 else -1
        drag.move(np.array([1., 0., 0.]), [-1, 0, 100 * side])
        force = drag.step(np.zeros(3), 1/60)
        assert np.linalg.norm(force) <= 12 + 1e-9
        assert np.linalg.norm(force - previous) <= 2.4/60 + 1e-9
        previous = force
    assert np.linalg.norm(previous) == pytest.approx(12)


def test_release_cancels_force_immediately_and_new_grab_starts_at_zero():
    drag = LimitedDrag()
    drag.begin([0, 0, 0], [0, 0, 0], [1, 0, 0])
    drag.move(np.array([1., 0., 0.]), [-1, 0, -1])
    assert np.linalg.norm(drag.step(np.zeros(3), 1/60)) > 0
    drag.end()
    assert drag.step(np.zeros(3), 1/60) == pytest.approx([0, 0, 0])
    drag.begin([0, 0, 2], [0, 0, 2.02], [1, 0, 0])
    assert drag.step(np.array([0, 0, 2]), 1/60) == pytest.approx([0, 0, 0])


def test_ray_parallel_to_drag_plane_cancels_capture():
    drag = LimitedDrag()
    drag.begin([0, 0, 0], [0, 0, 0], [1, 0, 0])
    drag.move(np.array([1., 0., 0.]), [0, 0, 1])
    assert not drag.active


def test_gui_routes_attached_fruit_exclusively_and_preserves_native_support_drag(monkeypatch):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Camera.Define(stage, "/Camera")
    monkeypatch.setitem(sys.modules, "omni.kit.viewport.utility",
                        SimpleNamespace(get_active_viewport=lambda: SimpleNamespace(camera_path="/Camera")))
    bridge = GuiDragBridge.__new__(GuiDragBridge)
    calls = []
    bridge.native = SimpleNamespace(update_interaction=lambda *args: calls.append(args))
    bridge.events = SimpleNamespace(MOUSE_DRAG_BEGAN=0, MOUSE_DRAG_CHANGED=1, MOUSE_DRAG_ENDED=2)
    bridge.stage, bridge.broken = stage, set()
    bridge.records = {"/fruit": {"fruit": "/fruit", "joint": "/joint"}}
    bridge.indices, bridge.coms = {"/fruit": 0}, np.zeros((1, 3))
    bridge.view = SimpleNamespace(get_world_poses=lambda **kw: (np.zeros((1, 3)), np.array([[1., 0, 0, 0]])))
    bridge.drag, bridge.capture, bridge.record = LimitedDrag(), None, None
    bridge.time_s, bridge.log, bridge.summary = 0., io.StringIO(), {"grabs": []}
    bridge.query = lambda *args: {"hit": True, "rigidBody": "/fruit", "collision": "/fruit/shape", "position": [0, 0, .01]}
    class NativeGuiVector:
        # omni.ui.scene.Vector3 supports iteration, but direct np.asarray fails.
        def __init__(self, values):
            self.values = values
        def __iter__(self):
            return iter(self.values)
        def __array__(self, *args, **kwargs):
            raise ValueError("setting an array element with a sequence")
    bridge.update_interaction(NativeGuiVector([0, 0, 1]), NativeGuiVector([0, 0, -1]), 0)
    bridge.update_interaction([0, 0, 1], [.1, 0, -1], 1)
    assert bridge.drag.active and calls == []
    assert not bridge.handle_break("/another_joint", .2)
    assert bridge.handle_break("/joint", .3)
    assert not bridge.drag.active
    bridge.update_interaction([0, 0, 1], [.2, 0, -1], 1)
    bridge.update_interaction([0, 0, 1], [.2, 0, -1], 2)
    assert calls == []  # Holding after detachment cannot start native grabbing.
    bridge.query = lambda *args: {"hit": True, "rigidBody": "/support"}
    for event in (0, 1, 2):
        bridge.update_interaction([0, 0, 1], [0, 0, -1], event)
    assert [call[-1] for call in calls] == [0, 1, 2]
