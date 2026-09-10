"""Replay recorded GUI rays through the fruit bridge in an offscreen app.

Use Isaac Python from repo root, --recorded-input GUI-INTERACTION.JSONL, then
the normal isaac_app arguments. This is a diagnostic replay, not desktop FPS
or manual acceptance. Selection is checked against each recorded begin event.
"""
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path.cwd() / "src"))
index = sys.argv.index("--recorded-input")
source = Path(sys.argv[index + 1])
del sys.argv[index:index + 2]
rows = [json.loads(line) for line in source.read_text().splitlines()]
events = [r for r in rows if r["event"] in ("begin", "move", "cancel")]

from exporterV2.fruit_interaction import GuiDragBridge, unit

original_init, original_step = GuiDragBridge.__init__, GuiDragBridge.before_step


class RecordedButtons:
    def __init__(self, native):
        self.native, self.down = native, False

    def __getattr__(self, name):
        return getattr(self.native, name)

    def get_mouse_value(self, *args):
        return float(self.down)

    def get_keyboard_value(self, keyboard, key):
        import carb.input
        return float(self.down and key in (carb.input.KeyboardInput.LEFT_SHIFT,
                                          carb.input.KeyboardInput.RIGHT_SHIFT))


def initialize(self, *args, **kwargs):
    original_init(self, *args, **kwargs)
    self.input = RecordedButtons(self.input)
    self.replay_index = 0
    self.summary["recorded_ray_replay"] = str(source.resolve())


def step(self, time_s, dt, broken):
    # GUI rays were delivered by render after the preceding physics step.
    while self.replay_index < len(events) and events[self.replay_index]["time_s"] + dt <= time_s + 1e-8:
        event = events[self.replay_index]
        self.replay_index += 1
        self.time_s = event["time_s"]
        if event["event"] == "begin":
            self.input.down = True
            self.update_interaction(event["origin"], event["direction"], self.events.MOUSE_DRAG_BEGAN)
            if self.record is None or self.record["fruit"] != event["body"]:
                raise RuntimeError(f"Recorded fruit selection mismatch: {event['body']}")
            self.drag.normal = unit(event["plane_normal"])
            self.summary["grabs"][-1]["plane_normal"] = self.drag.normal.tolist()
        elif event["event"] == "move":
            self.update_interaction(event["origin"], event["direction"], self.events.MOUSE_DRAG_CHANGED)
        elif event.get("reason") not in ("joint_break", "monitor_exit"):
            self.input.down = False
            self.cancel("recorded_release")
    return original_step(self, time_s, dt, broken)


GuiDragBridge.__init__, GuiDragBridge.before_step = initialize, step
import isaacsim
original_app = isaacsim.SimulationApp
isaacsim.SimulationApp = lambda config: original_app({**config, "headless": True})
from exporterV2.isaac_app import main
code = main()
sys.stdout.flush()
sys.stderr.flush()
os._exit(code)
