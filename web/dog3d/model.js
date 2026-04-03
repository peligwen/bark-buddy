// Dog model builder — body, head, legs, ultrasonic sensors, tail
import { S, BODY_L, BODY_W, BODY_H, UPPER_LEN, UPPER_R, LOWER_LEN, LOWER_R,
         FOOT_R, HIP_OFFSET_X, HIP_OFFSET_Z, HIP_OFFSET_Y,
         STAND_HIP, STAND_KNEE, COL, mat, state } from './state.js';

export function buildDog(group) {
    var bodyMat = mat(COL.body, { roughness: 0.45, metalness: 0.1 });
    var legMat = mat(COL.leg, { roughness: 0.6, metalness: 0.15 });
    var headMat = mat(COL.head, { roughness: 0.4, metalness: 0.1 });
    var accentMat = mat(COL.accent, { roughness: 0.5, metalness: 0.2 });
    var eyeMat = mat(COL.eye, { emissive: COL.eye, ei: 0.5 });
    var sensorMat = mat(COL.sensor, { metalness: 0.4, roughness: 0.3 });

    // Body chassis
    var body = new THREE.Mesh(new THREE.BoxGeometry(BODY_L, BODY_H, BODY_W), bodyMat);
    body.castShadow = true;
    group.add(body);

    // Top accent plate
    var plate = new THREE.Mesh(new THREE.BoxGeometry(BODY_L * 0.8, BODY_H * 0.15, BODY_W * 0.7), accentMat);
    plate.position.y = BODY_H * 0.55;
    group.add(plate);

    // Head block
    var headW = BODY_W * 0.65, headH = BODY_H * 1.0, headL = BODY_L * 0.28;
    var head = new THREE.Mesh(new THREE.BoxGeometry(headL, headH, headW), headMat);
    head.position.set(BODY_L / 2 + headL * 0.35, BODY_H * 0.3, 0);
    head.castShadow = true;
    group.add(head);

    // Snout
    var snout = new THREE.Mesh(new THREE.BoxGeometry(headL * 0.6, headH * 0.6, headW * 0.7), bodyMat);
    snout.position.set(BODY_L / 2 + headL * 0.85, BODY_H * 0.1, 0);
    group.add(snout);

    // Eyes
    var eyeR = 0.006 * S;
    var eyeGeo = new THREE.SphereGeometry(eyeR, 8, 8);
    [-1, 1].forEach(function (side) {
        var eye = new THREE.Mesh(eyeGeo, eyeMat);
        eye.position.set(BODY_L / 2 + headL * 0.6, BODY_H * 0.45, side * headW * 0.28);
        group.add(eye);
    });

    // Ears
    [-1, 1].forEach(function (side) {
        var ear = new THREE.Mesh(new THREE.BoxGeometry(headL * 0.3, headH * 0.5, headW * 0.2), accentMat);
        ear.position.set(BODY_L / 2 + headL * 0.15, BODY_H * 0.85, side * headW * 0.42);
        ear.rotation.z = 0.2;
        group.add(ear);
    });

    // Tail + flag
    var tail = new THREE.Mesh(new THREE.BoxGeometry(BODY_L * 0.12, BODY_H * 0.4, BODY_H * 0.4), accentMat);
    tail.position.set(-BODY_L / 2 - BODY_L * 0.06, BODY_H * 0.3, 0);
    tail.rotation.z = -0.3;
    group.add(tail);

    var flagPole = new THREE.Mesh(
        new THREE.CylinderGeometry(0.003 * S, 0.003 * S, 0.08 * S, 6), mat(0x444444));
    flagPole.position.set(-BODY_L / 2 - BODY_L * 0.1, BODY_H * 0.7, 0);
    group.add(flagPole);

    var flag = new THREE.Mesh(
        new THREE.PlaneGeometry(0.04 * S, 0.025 * S),
        new THREE.MeshBasicMaterial({ color: 0xdd2222, side: THREE.DoubleSide }));
    flag.position.set(-BODY_L / 2 - BODY_L * 0.1 - 0.02 * S, BODY_H * 0.95, 0);
    group.add(flag);

    // Ultrasonic sensors
    var usGeo = new THREE.CylinderGeometry(0.02 * S, 0.02 * S, 0.02 * S, 8);
    [-1, 1].forEach(function (side) {
        var us = new THREE.Mesh(usGeo, sensorMat);
        us.position.set(BODY_L / 2 + headL * 1.05, BODY_H * 0.1, side * 0.01 * S);
        us.rotation.z = Math.PI / 2;
        group.add(us);
    });

    // Ultrasonic beam
    var beamGeo = new THREE.ConeGeometry(0.15 * S, 1.0 * S, 8, 1, true);
    var beamMat = new THREE.MeshBasicMaterial({
        color: COL.beam, transparent: true, opacity: 0.12,
        side: THREE.DoubleSide, depthWrite: false,
    });
    state.ultraBeam = new THREE.Mesh(beamGeo, beamMat);
    state.ultraBeam.rotation.z = Math.PI / 2;
    state.ultraBeam.visible = false;
    state.ultraBeam.userData.sensorX = BODY_L / 2 + headL * 1.05;
    state.ultraBeam.userData.sensorY = BODY_H * 0.1;
    state.ultraBeam.position.set(state.ultraBeam.userData.sensorX + 0.5 * S, state.ultraBeam.userData.sensorY, 0);
    group.add(state.ultraBeam);

    // Articulated legs
    var legDefs = [
        { name: "fl", x: HIP_OFFSET_X, z: HIP_OFFSET_Z },
        { name: "fr", x: HIP_OFFSET_X, z: -HIP_OFFSET_Z },
        { name: "rl", x: -HIP_OFFSET_X, z: HIP_OFFSET_Z },
        { name: "rr", x: -HIP_OFFSET_X, z: -HIP_OFFSET_Z },
    ];
    var jointMat = mat(COL.joint, { roughness: 0.5, metalness: 0.3 });

    legDefs.forEach(function (def) {
        var leg = buildLeg(def.name, legMat, jointMat);
        leg.hipPivot.position.set(def.x, HIP_OFFSET_Y, def.z);
        group.add(leg.hipPivot);
        state.legs[def.name] = leg;
    });

    setAllLegs(STAND_HIP, STAND_KNEE);
}

function buildLeg(name, legMat, footMat) {
    var hipPivot = new THREE.Group();
    var upperGeo = new THREE.CylinderGeometry(UPPER_R, UPPER_R, UPPER_LEN, 8);
    var upperMesh = new THREE.Mesh(upperGeo, legMat);
    upperMesh.position.y = -UPPER_LEN / 2;
    upperMesh.castShadow = true;
    hipPivot.add(upperMesh);

    var jointGeo = new THREE.SphereGeometry(UPPER_R * 1.5, 8, 8);
    hipPivot.add(new THREE.Mesh(jointGeo, footMat));

    var kneePivot = new THREE.Group();
    kneePivot.position.y = -UPPER_LEN;
    hipPivot.add(kneePivot);

    kneePivot.add(new THREE.Mesh(jointGeo, footMat));

    var lowerGeo = new THREE.CylinderGeometry(LOWER_R, LOWER_R, LOWER_LEN, 8);
    var lowerMesh = new THREE.Mesh(lowerGeo, legMat);
    lowerMesh.position.y = -LOWER_LEN / 2;
    lowerMesh.castShadow = true;
    kneePivot.add(lowerMesh);

    var footGeo = new THREE.SphereGeometry(FOOT_R, 8, 8);
    var footMesh = new THREE.Mesh(footGeo, footMat);
    footMesh.position.y = -LOWER_LEN;
    footMesh.castShadow = true;
    kneePivot.add(footMesh);

    return { hipPivot, kneePivot, upperMesh, lowerMesh, footMesh };
}

export function setLeg(name, hipAngle, kneeAngle) {
    var leg = state.legs[name];
    if (!leg) return;
    leg.hipPivot.rotation.z = hipAngle;
    leg.kneePivot.rotation.z = kneeAngle;
}

export function setAllLegs(hip, knee) {
    ["fl", "fr", "rl", "rr"].forEach(function (name) { setLeg(name, hip, knee); });
}
