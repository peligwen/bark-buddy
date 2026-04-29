// Pose holding + named-pose interpolation, plus sim-joint application.
//
// We DO NOT synthesise a trot here. When the dog is engaged, firmware streams
// telem_joints (~20 Hz) and the viz interpolates those directly via
// applySimJoints(). When the dog is disengaged or telemetry is stale, we
// freeze on the last commanded pose (or STAND) — never invent motion that
// the firmware isn't actually doing.

import { state, STAND_HIP, STAND_KNEE, UPPER_LEN, LOWER_LEN,
         HIP_OFFSET_Y, FOOT_R, standingHeight } from './state.js';
import { setLeg, setAllLegs } from './model.js';

// Named poses (validated against firmware/test/pose_generator).
const POSES = {
    stand:    [0.3000, -0.6000, 0.3000, -0.6000, 0.3000, -0.6000, 0.3000, -0.6000],
    tall:     [0.1500, -0.5406, 0.1500, -0.5406, 0.1500, -0.5406, 0.1500, -0.5406],
    crouch:   [0.5000, -0.8000, 0.5000, -0.8000, 0.5000, -0.8000, 0.5000, -0.8000],
    rest:     [0.4000, -0.5629, 0.4000, -0.5629, 0.4000, -0.5629, 0.4000, -0.5629],
    alert:    [0.2000, -0.5691, 0.2000, -0.5691, 0.4000, -0.5629, 0.4000, -0.5629],
    sit:      [0.0500, -0.4639, 0.0500, -0.4639, 1.0000, -1.4000, 1.0000, -1.4000],
    lie_down: [0.8000, -1.2000, 0.8000, -1.2000, 0.8000, -1.2000, 0.8000, -1.2000],
    play_bow: [0.7000, -1.0000, 0.7000, -1.0000, 0.1500, -0.5406, 0.1500, -0.5406],
};

export function setPose(name) {
    const angles = POSES[name];
    if (!angles) return;
    state.currentPoseName = name;
    state.targetPose = angles.slice();
    state.activePose = angles.slice();
}

export function clearPose() {
    state.activePose = null;
    state.targetPose = null;
    state.currentPoseName = "stand";
}

// Compute body height by finding the lowest foot in the pose.
function poseBodyHeight(angles) {
    let minFootY = 0;
    for (let i = 0; i < 4; i++) {
        const hip = angles[i * 2];
        const knee = angles[i * 2 + 1];
        let footY = -(UPPER_LEN * Math.cos(hip) + LOWER_LEN * Math.cos(hip + Math.abs(knee)));
        footY += HIP_OFFSET_Y;
        if (footY < minFootY) minFootY = footY;
    }
    return -minFootY + FOOT_R;
}

function animateAction() {
    const holdAngles = state.targetPose || state.activePose;
    if (!holdAngles) return false;

    const legNames = ["fl", "fr", "rl", "rr"];
    for (let i = 0; i < 4; i++) {
        const leg = state.legs[legNames[i]];
        if (!leg) continue;
        const th = holdAngles[i * 2];
        const tk = holdAngles[i * 2 + 1];
        if (state.targetPose) {
            const ch = leg.hipPivot.rotation.z;
            const ck = leg.kneePivot.rotation.z;
            const dh = th - ch, dk = tk - ck;
            if (Math.abs(dh) > 0.005 || Math.abs(dk) > 0.005) {
                setLeg(legNames[i], ch + dh * 0.1, ck + dk * 0.1);
            } else {
                setLeg(legNames[i], th, tk);
            }
        } else {
            setLeg(legNames[i], th, tk);
        }
    }

    if (state.targetPose) {
        let done = true;
        for (let j = 0; j < 4; j++) {
            const l = state.legs[legNames[j]];
            if (!l) continue;
            if (Math.abs(l.hipPivot.rotation.z - state.targetPose[j*2]) > 0.005 ||
                Math.abs(l.kneePivot.rotation.z - state.targetPose[j*2+1]) > 0.005) {
                done = false;
                break;
            }
        }
        if (done) state.targetPose = null;  // keep activePose for holding
    }

    state.bodyBounce = poseBodyHeight(holdAngles) - standingHeight();
    return true;
}

// Render the model when telem_joints is stale. Holds last pose or STAND.
// No synthesised trot — the firmware is the source of truth for motion.
export function animateHold() {
    state.bodyBounce = 0;
    if (animateAction()) return;
    setAllLegs(STAND_HIP, STAND_KNEE);
}

export function applySimJoints() {
    if (!state.simJoints) return;
    ["fl", "fr", "rl", "rr"].forEach(function (name) {
        const j = state.simJoints[name];
        if (j) setLeg(name, j.hip, j.knee);
    });
}
