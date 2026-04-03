// Shared state and constants for the 3D dog visualization.
// All dog3d submodules import from here.

// --- URDF dimensions (meters, scaled 10x for visibility) ---
export const S = 10;
// Dimensions from official MechDog spec (214x126x138mm standing)
export const BODY_L = 0.170 * S, BODY_W = 0.060 * S, BODY_H = 0.035 * S;
export const UPPER_LEN = 0.055 * S, UPPER_R = 0.008 * S;
export const LOWER_LEN = 0.060 * S, LOWER_R = 0.006 * S;
export const FOOT_R = 0.008 * S;
export const HIP_OFFSET_X = 0.085 * S;
export const HIP_OFFSET_Z = 0.030 * S;
export const HIP_OFFSET_Y = -0.025 * S;
export const STAND_HIP = 0.524;   // ~30 degrees
export const STAND_KNEE = -0.611; // ~-35 degrees

export const COL = {
    body: 0xd4a017, accent: 0x1a1a1a, leg: 0x2c2c2c, head: 0xe2b52e,
    eye: 0x33ff66, ground: 0x1a1d24, sensor: 0x888888, beam: 0x33aaff,
    joint: 0x444444, grid: 0x2a2f3a, sceneBg: 0x1e222b,
};

// Mutable shared state — accessed by all submodules
export const state = {
    scene: null,
    camera: null,
    renderer: null,
    dogGroup: null,
    container: null,
    animationId: null,
    legs: {},

    // Camera
    isDragging: false,
    prevMouse: { x: 0, y: 0 },
    cameraAngle: { thetaOffset: 0, phi: Math.PI / 5, radius: 8 },
    camYaw: 0,
    camTargetX: 0,
    camTargetZ: 0,

    // Telemetry targets (smoothed)
    targetPitch: 0, targetRoll: 0,
    currentPitch: 0, currentRoll: 0,
    targetX: 0, targetZ: 0, targetYaw: 0,
    currentX: 0, currentZ: 0, currentYaw: 0,
    currentMotion: "stop",
    currentAction: null,
    ultraDistance: null,
    walkPhase: 0,
    isFallen: false,

    // Gait
    bodyBounce: 0,

    // Pose
    currentPoseName: "stand",
    activePose: null,
    targetPose: null,

    // Sim joints
    simJoints: null,

    // Ultrasonic
    ultraBeam: null,
    ultraHit: null,
};

// Helper: material factory
export function mat(color, opts) {
    var m = new THREE.MeshStandardMaterial({ color: color, roughness: 0.6, metalness: 0.2 });
    if (opts) {
        if (opts.emissive) { m.emissive = new THREE.Color(opts.emissive); m.emissiveIntensity = opts.ei || 0.8; }
        if (opts.roughness != null) m.roughness = opts.roughness;
        if (opts.metalness != null) m.metalness = opts.metalness;
    }
    return m;
}

export function standingHeight() {
    var hipA = STAND_HIP, kneeA = STAND_KNEE;
    var footDy = UPPER_LEN * Math.cos(hipA) + LOWER_LEN * Math.cos(hipA + Math.abs(kneeA));
    return footDy - HIP_OFFSET_Y + FOOT_R;
}
