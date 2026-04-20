// cmd_transform payloads for named body poses.
// All values are relative to neutral standing (empty object = stand).
// sit/lie_down/wave require per-leg firmware pose tables — deferred.
export var TRANSFORM_POSES = {
    stand:      {},
    tall:       { z:  15 },
    crouch:     { z: -15 },
    look_up:    { pitch:  12 },
    look_down:  { pitch: -12 },
    lean_left:  { roll:  10 },
    lean_right: { roll: -10 },
    bow:        { pitch: -15, z: -8 },
};
