"""Offscreen native UI-event smoke probe; run from repo root with Isaac Python.

Accepts isaac_app arguments, including --fruit-experiment CONFIG and --duration
15. Always uses a headless app. It aims at r5 fruit08 but accepts the actual
raycast hit (fruit07 in the recorded scene); this checks GUI routing and visual
feedback, not the exact-fruit comparison or the 60-second acceptance protocol.
Snapshots are written beside CONFIG. Never use this to measure desktop FPS.
"""
import sys, os
from pathlib import Path
probe_output = Path(sys.argv[sys.argv.index('--fruit-experiment')+1]).parent
sys.path.insert(0, str(Path.cwd()/'src'))
from exporterV2.fruit_interaction import GuiDragBridge
original_init=GuiDragBridge.__init__
original_step=GuiDragBridge.before_step
original_close=GuiDragBridge.close

def init(self,*args,**kwargs):
    original_init(self,*args,**kwargs)
    import carb.input
    from omni.physxui import get_physicsui_instance
    self.probe_overlay=get_physicsui_instance()._viewport_overlays
    self.probe_provider=carb.input.acquire_input_provider()
    self.probe_step=0
    def mouse(e):
        if e.type != carb.input.MouseEventType.MOVE:
            print('[INPUT_PROBE] mouse',e.type,'clip',self.probe_overlay._clip_vstack.content_clipping,'dragging',self.probe_overlay._mouse_interaction_simulation_dragging,flush=True)
        return True
    def key(e):
        if e.input == carb.input.KeyboardInput.LEFT_SHIFT:
            print('[INPUT_PROBE] shift',e.type,'clip',self.probe_overlay._clip_vstack.content_clipping,flush=True)
        return True
    self.probe_mouse=self.input.subscribe_to_mouse_events(self.window.get_mouse(),mouse)
    self.probe_key=self.input.subscribe_to_keyboard_events(self.window.get_keyboard(),key)

def step(self,time_s,dt,broken):
    import carb.input as ci
    import omni.ui as ui
    from omni.kit.viewport.utility import get_active_viewport_window
    w=get_active_viewport_window(); f=w.get_frame("omni.physx.ui.root_frame")
    n=self.probe_step; self.probe_step+=1
    if n==90:
        from isaacsim.core.utils.viewports import set_camera_view
        from exporterV2.fruit_interaction import pick_ray
        body='/World/TerminalBodies/Truss_r5_o0_g421786_tomato_08'
        center=self.center(body)
        eye,direction,hit,attempts=pick_ray(center,body,self.query)
        eye=center+3*(eye-center)
        set_camera_view(eye=eye,target=center,camera_prim_path=str(w.viewport_api.camera_path))
        print('[INPUT_PROBE] target',body,'eye',eye,'center',center,flush=True)
    if n==120:
        self.probe_xy=(f.screen_position_x+f.computed_width*.5,f.screen_position_y+f.computed_height*.5)
        print('[INPUT_PROBE] frame',self.probe_xy,'window',ui.Workspace.get_main_window_width(),ui.Workspace.get_main_window_height(),flush=True)
        self.probe_provider.buffer_keyboard_key_event(self.window.get_keyboard(),ci.KeyboardEventType.KEY_PRESS,ci.KeyboardInput.LEFT_SHIFT,ci.KEYBOARD_MODIFIER_FLAG_SHIFT)
    if n in (125,130,600) or 135<=n<=430:
        event=ci.MouseEventType.LEFT_BUTTON_DOWN if n==130 else ci.MouseEventType.LEFT_BUTTON_UP if n==600 else ci.MouseEventType.MOVE
        x,y=self.probe_xy; x+=max(0,min(n-135,295))*1.5
        dpi=ui.Workspace.get_dpi_scale(); pos=(x*dpi,y*dpi)
        self.probe_provider.buffer_mouse_event(self.window.get_mouse(),event,(pos[0]/ui.Workspace.get_main_window_width(),pos[1]/ui.Workspace.get_main_window_height()),ci.KEYBOARD_MODIFIER_FLAG_SHIFT,pos)
    if n==605:
        self.probe_provider.buffer_keyboard_key_event(self.window.get_keyboard(),ci.KeyboardEventType.KEY_RELEASE,ci.KeyboardInput.LEFT_SHIFT,0)
    result = original_step(self,time_s,dt,broken)
    if n in (300, 420, 500):
        import omni.kit.renderer_capture as rc
        capture = rc.acquire_renderer_capture_interface()
        capture.capture_next_frame_swapchain(str(probe_output / f'feedback-{n}.png'), self.window)
    return result

def close(self,t):
    self.input.unsubscribe_to_mouse_events(self.window.get_mouse(),self.probe_mouse)
    self.input.unsubscribe_to_keyboard_events(self.window.get_keyboard(),self.probe_key)
    original_close(self,t)
GuiDragBridge.__init__=init
GuiDragBridge.before_step=step
GuiDragBridge.close=close
import isaacsim
original_app=isaacsim.SimulationApp
isaacsim.SimulationApp=lambda config: original_app({**config, 'headless': True})
from exporterV2.isaac_app import main
code=main();sys.stdout.flush();sys.stderr.flush();os._exit(code)
