// Gait animation, named poses, and sim joint application.
import { state, S, STAND_HIP, STAND_KNEE, UPPER_LEN, LOWER_LEN,
         HIP_OFFSET_Y, FOOT_R, standingHeight } from './state.js';
import { setLeg, setAllLegs } from './model.js';

// Named poses (validated by firmware/test/pose_generator)
export var POSES = {
    stand:    [0.3000, -0.6000, 0.3000, -0.6000, 0.3000, -0.6000, 0.3000, -0.6000],
    tall:     [0.1500, -0.5406, 0.1500, -0.5406, 0.1500, -0.5406, 0.1500, -0.5406],
    crouch:   [0.5000, -0.8000, 0.5000, -0.8000, 0.5000, -0.8000, 0.5000, -0.8000],
    rest:     [0.4000, -0.5629, 0.4000, -0.5629, 0.4000, -0.5629, 0.4000, -0.5629],
    alert:    [0.2000, -0.5691, 0.2000, -0.5691, 0.4000, -0.5629, 0.4000, -0.5629],
    sit:      [0.0500, -0.4639, 0.0500, -0.4639, 1.0000, -1.4000, 1.0000, -1.4000],
    lie_down: [0.8000, -1.2000, 0.8000, -1.2000, 0.8000, -1.2000, 0.8000, -1.2000],
    play_bow: [0.7000, -1.0000, 0.7000, -1.0000, 0.1500, -0.5406, 0.1500, -0.5406],
};

export function getPoseNames() {
    return Object.keys(POSES);
}

export function setPose(name) {
    var angles = POSES[name];
    if (!angles) return;
    state.currentPoseName = name;
    state.targetPose = angles.slice();
    state.activePose = angles.slice();
    state.currentAction = null;
    state.currentMotion = "stop";
}

export function clearPose() {
    state.activePose = null;
    state.targetPose = null;
    state.currentPoseName = "stand";
}

// Compute body height for a given pose by finding the lowest foot
function poseBodyHeight(angles) {
    // Forward kinematics: foot Y relative to hip
    // foot_y = -upper*cos(hip) - lower*cos(hip+|knee|)
    var minFootY = 0;
    for (var i = 0; i < 4; i++) {
        var hip = angles[i * 2];
        var knee = angles[i * 2 + 1];
        var footY = -(UPPER_LEN * Math.cos(hip) + LOWER_LEN * Math.cos(hip + Math.abs(knee)));
        footY += HIP_OFFSET_Y;
        if (footY < minFootY) minFootY = footY;
    }
    return -minFootY + FOOT_R;
}

function animateAction() {
    // Hold active pose or interpolate to target
    var holdAngles = state.targetPose || state.activePose;
    if (holdAngles) {
        var legNames = ["fl", "fr", "rl", "rr"];
        for (var i = 0; i < 4; i++) {
            var leg = state.legs[legNames[i]];
            if (!leg) continue;
            var th = holdAngles[i * 2];
            var tk = holdAngles[i * 2 + 1];
            if (state.targetPose) {
                // Interpolate
                var ch = leg.hipPivot.rotation.z;
                var ck = leg.kneePivot.rotation.z;
                var dh = th - ch, dk = tk - ck;
                if (Math.abs(dh) > 0.005 || Math.abs(dk) > 0.005) {
                    setLeg(legNames[i], ch + dh * 0.1, ck + dk * 0.1);
                } else {
                    setLeg(legNames[i], th, tk);
                }
            } else {
                // Hold
                setLeg(legNames[i], th, tk);
            }
        }

        // Check if interpolation is done
        if (state.targetPose) {
            var done = true;
            for (var j = 0; j < 4; j++) {
                var l = state.legs[legNames[j]];
                if (!l) continue;
                if (Math.abs(l.hipPivot.rotation.z - state.targetPose[j*2]) > 0.005 ||
                    Math.abs(l.kneePivot.rotation.z - state.targetPose[j*2+1]) > 0.005) {
                    done = false;
                    break;
                }
            }
            if (done) state.targetPose = null; // keep activePose for holding
        }

        // Adjust body height for the pose
        state.bodyBounce = poseBodyHeight(holdAngles) - standingHeight();
        return;
    }

    var a = state.currentAction;
    if (a === 1) {
        setLeg("fl", -0.8, -1.2);
        setLeg("fr", STAND_HIP, STAND_KNEE);
        setLeg("rl", STAND_HIP * 1.3, STAND_KNEE * 0.8);
        setLeg("rr", STAND_HIP * 1.3, STAND_KNEE * 0.8);
    } else {
        setAllLegs(STAND_HIP, STAND_KNEE);
    }
}

export function animateGait(dt) {
    var speed = 0;
    if (state.currentMotion === "forward") speed = 6;
    else if (state.currentMotion === "backward") speed = -4;
    else if (state.currentMotion === "left" || state.currentMotion === "right") speed = 3;

    if (speed === 0) {
        state.walkPhase = 0;
        state.bodyBounce = 0;
        // Pose interpolation or action takes priority over standing reset
        if (state.targetPose || state.currentAction != null) {
            animateAction();
            return;
        }
        setAllLegs(STAND_HIP, STAND_KNEE);
        return;
    }

    if (state.currentAction != null || state.targetPose) {
        animateAction();
        state.bodyBounce = 0;
        return;
    }

    state.walkPhase += speed * dt;

    // Trot gait: diagonal pairs move together
    var hipAmp = 0.22;   // hip swing amplitude (radians)
    var kneeAmp = 0.35;  // knee lift amplitude
    var kneeBias = 0.1;  // extra knee bend to keep feet above ground

    var phA = Math.sin(state.walkPhase);
    var phB = Math.sin(state.walkPhase + Math.PI);

    // Knee lifts during swing (positive phase), extra bend during stance
    var kneeA = STAND_KNEE - kneeBias - kneeAmp * Math.abs(phA);
    var kneeB = STAND_KNEE - kneeBias - kneeAmp * Math.abs(phB);

    setLeg("fl", STAND_HIP + hipAmp * phA, kneeA);
    setLeg("rr", STAND_HIP + hipAmp * phA, kneeA);
    setLeg("fr", STAND_HIP + hipAmp * phB, kneeB);
    setLeg("rl", STAND_HIP + hipAmp * phB, kneeB);

    // Body dynamics
    // Double-frequency bounce (body rises at mid-swing of each pair)
    state.bodyBounce = 0.015 * S * Math.abs(Math.sin(state.walkPhase * 2));
}

export function applySimJoints() {
    if (!state.simJoints) return;
    ["fl", "fr", "rl", "rr"].forEach(function (name) {
        var j = state.simJoints[name];
        if (j) {
            setLeg(name, j.hip, j.knee);
        }
    });
}
