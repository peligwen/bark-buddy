// Ultrasonic beam visualization and hit-point beacon.
import { state, S, BODY_L, BODY_H, COL, standingHeight } from './state.js';

var WALL_HEIGHT = 2.0; // scene units — must match walls.js

export function initUltraHit() {
    // Tall vertical beacon at hit point — sticks above walls
    var BEACON_HEIGHT = WALL_HEIGHT + 1.5;
    state.ultraHit = new THREE.Group();
    state.ultraHit.visible = false;
    state.scene.add(state.ultraHit);

    // Vertical line (thin cylinder from ground to above wall height)
    var poleGeo = new THREE.CylinderGeometry(0.008 * S, 0.008 * S, BEACON_HEIGHT, 6);
    var poleMat = new THREE.MeshBasicMaterial({ color: 0xff3333 });
    var pole = new THREE.Mesh(poleGeo, poleMat);
    pole.position.y = BEACON_HEIGHT / 2;
    state.ultraHit.add(pole);

    // Diamond marker at top
    var diamondGeo = new THREE.OctahedronGeometry(0.04 * S, 0);
    var diamondMat = new THREE.MeshBasicMaterial({ color: 0xff3333 });
    var diamond = new THREE.Mesh(diamondGeo, diamondMat);
    diamond.position.y = BEACON_HEIGHT;
    state.ultraHit.add(diamond);

    // Ground ring at base
    var ringGeo = new THREE.RingGeometry(0.04 * S, 0.06 * S, 16);
    var ringMat = new THREE.MeshBasicMaterial({
        color: 0xff3333, side: THREE.DoubleSide,
        transparent: true, opacity: 0.5,
    });
    var ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.02;
    state.ultraHit.add(ring);

    // Store refs for color updates
    state.ultraHit.userData.pole = pole;
    state.ultraHit.userData.diamond = diamond;
}

export function updateUltraBeam() {
    if (state.ultraDistance == null || state.ultraDistance > 2500) {
        if (state.ultraBeam) state.ultraBeam.visible = false;
        if (state.ultraHit) state.ultraHit.visible = false;
        return;
    }
    if (!state.ultraBeam) return;
    state.ultraBeam.visible = true;

    // Convert mm to scene units, clamp
    var len = (state.ultraDistance / 1000) * S;
    var maxLen = 3.0 * S;
    len = Math.min(len, maxLen);

    // Cone spans from sensor to wall hit point
    // len = center-to-wall distance; sx = center-to-sensor offset
    var sx = state.ultraBeam.userData.sensorX;
    var sy = state.ultraBeam.userData.sensorY;
    var beamLen = Math.max(0.01, len - sx); // sensor-to-wall distance

    var lengthScale = beamLen / (1.0 * S);
    state.ultraBeam.scale.y = lengthScale;
    var widthScale = 0.5 + 0.5 * (beamLen / maxLen);
    state.ultraBeam.scale.x = widthScale;
    state.ultraBeam.scale.z = widthScale;

    // Cone center is halfway between sensor and wall
    state.ultraBeam.position.set(sx + (beamLen / 2), sy, 0);

    // Color by distance
    if (state.ultraDistance < 150) {
        state.ultraBeam.material.color.setHex(0xe94560);
        state.ultraBeam.material.opacity = 0.25;
    } else if (state.ultraDistance < 400) {
        state.ultraBeam.material.color.setHex(0xf0a500);
        state.ultraBeam.material.opacity = 0.18;
    } else {
        state.ultraBeam.material.color.setHex(COL.beam);
        state.ultraBeam.material.opacity = 0.12;
    }

    // Hit point: compute directly from target (non-interpolated) position + heading
    // Distance is from dog center (not sensor), so don't add sensor offset
    if (state.ultraHit) {
        var cosY = Math.cos(state.targetYaw);
        var sinY = Math.sin(state.targetYaw);
        var hitWorldX = state.targetX + len * cosY;
        var hitWorldZ = state.targetZ - len * sinY;
        state.ultraHit.position.set(hitWorldX, 0, hitWorldZ);
        state.ultraHit.visible = true;

        // Color beacon by distance
        var hitColor = state.ultraDistance < 150 ? 0xff2222 :
                       state.ultraDistance < 400 ? 0xff8800 : 0x33aaff;
        state.ultraHit.userData.pole.material.color.setHex(hitColor);
        state.ultraHit.userData.diamond.material.color.setHex(hitColor);
    }
}
