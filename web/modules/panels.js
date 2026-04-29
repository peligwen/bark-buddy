// web/modules/panels.js — barrel re-export.
//
// The actual panels live in panels/<name>.js — split out per-panel so
// adding a new one (eg SLAM debug) doesn't bloat a single 300-line file.
// This barrel exists for one reason: app.module.js imports a flat list of
// names and there's no value in spreading those imports across six lines.

export { setupBatteryGraph, recordBattery }                  from './panels/battery.js';
export { setupOtaPanel, updateOtaStatus }                    from './panels/ota.js';
export { initOffsetPanel, updateServoPins }                  from './panels/servo_offset.js';
export { initGaitPanel }                                     from './panels/gait.js';
export { initTransformPanel, setTransformSliders }           from './panels/transform.js';
export { initServoNudgePanel }                               from './panels/servo_nudge.js';
