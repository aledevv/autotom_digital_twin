"""Physical acceptance checks, separate from process/runtime success."""
import numpy as np


def evaluate(data, c, scenario, duration):
    if len(data) == 0 or not np.isfinite(data).all():
        return dict(status="failed", reason="Missing or nonfinite trace")
    time, phase = data[:, 0], data[:, 1]
    baseline = (time > 1.5) & (time < 2)
    recovered = (phase > 17) & (phase < 19.9)
    press = (phase > 4) & (phase < 5)
    drop = (phase > 9) & (phase < 12)
    tip0 = data[baseline, 3:6].mean(axis=0)
    local0 = data[baseline, 6:9].mean(axis=0)
    local_motion = np.linalg.norm(data[:, 6:9]-local0, axis=1)
    world_motion = np.linalg.norm(data[:, 3:6]-tip0, axis=1)
    recovery = float(world_motion[recovered].max()) if recovered.any() else None
    force = data[:, 10]
    checks = dict(complete=bool(time[-1] >= duration-1/c.hz-1e-6),
                  finite=True, joint_constraints=bool(data[:, 9].max() < .0003),
                  no_collapse=bool(data[:, 5].min() > c.height-c.length*.8),
                  rest_not_rubber=bool(abs(tip0[2]-c.height) < c.length*.1),
                  recovery=bool(recovery is not None and recovery < .0015))
    cycle_reports = []
    for cycle in range(int(duration//20)):
        within = (time > cycle*20) & (time <= (cycle+1)*20+1e-5)
        settled = within & recovered
        pressed = within & press
        dropped = within & drop
        cycle_checks = dict(recovery=bool(settled.any() and world_motion[settled].max() < .0015),
                            settled_motion=bool(settled.any() and np.ptp(data[settled, 3:6], axis=0).max() < .0002))
        if scenario in ("press", "cycle"):
            cycle_checks["press"] = bool(pressed.any() and force[pressed].max() > .001 and local_motion[pressed].max() > .004)
        if scenario in ("drop", "cycle"):
            cycle_checks["drop"] = bool(dropped.any() and force[dropped].max() > .001 and local_motion[dropped].max() > .001)
        cycle_reports.append(dict(cycle=cycle+1, checks=cycle_checks))
    checks["all_cycles"] = bool(cycle_reports and all(all(x["checks"].values()) for x in cycle_reports))
    if data.shape[1] >= 18:
        expected_z = c.height+c.probe_radius+.025-(.025+c.press_depth)*data[:, 2]
        checks["probe_tracks_target"] = bool(np.max(abs(data[:, 17]-expected_z)) < .0001)
    if scenario in ("press", "cycle"):
        checks.update(press_contact=bool(force[press].max() > .001),
                      local_bending=bool(local_motion[press].max() > .004),
                      limited_bending=bool(local_motion[press].max() < c.length*.6))
    if scenario in ("drop", "cycle"):
        checks.update(drop_contact=bool(force[drop].max() > .001),
                      drop_bending=bool(local_motion[drop].max() > .001))
    return dict(status="passed" if all(checks.values()) else "failed", checks=checks, cycles=cycle_reports,
                rest_sag_mm=float((c.height-tip0[2])*1000),
                press_local_deflection_mm=float(local_motion[press].max()*1000) if press.any() else None,
                drop_local_deflection_mm=float(local_motion[drop].max()*1000) if drop.any() else None,
                recovery_error_mm=recovery*1000 if recovery is not None else None,
                max_joint_error_mm=float(data[:, 9].max()*1000),
                max_contact_n=float(force.max()), min_tip_height_m=float(data[:, 5].min()),
                max_petiole_angle_deg=float(np.rad2deg(data[:, 14].max())),
                validation="Numerical interactive prototype, not a biomechanical calibration")
