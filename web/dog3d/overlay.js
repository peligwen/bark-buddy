// Debug overlay: wireframe body, joint indicators, IMU axes.
import { state, S, BODY_L, BODY_H, BODY_W, UPPER_LEN, LOWER_LEN } from './state.js';

var overlayVisible = false;
var overlayParts = [];  // all overlay objects to toggle visibility
var jointMarkers = {};

function makeArrow(x1, y1, z1, x2, y2, z2, color) {
    var group = new THREE.Group();
    // Shaft — cylinder
    var dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
    var len = Math.sqrt(dx * dx + dy * dy + dz * dz);
    var shaft = new THREE.Mesh(
        new THREE.CylinderGeometry(0.008 * S, 0.008 * S, len, 6),
        new THREE.MeshBasicMaterial({ color: color })
    );
    shaft.position.set((x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2);
    shaft.lookAt(x2, y2, z2);
    shaft.rotateX(Math.PI / 2);
    group.add(shaft);
    // Tip sphere
    var tip = new THREE.Mesh(
        new THREE.SphereGeometry(0.02 * S, 8, 8),
        new THREE.MeshBasicMaterial({ color: color })
    );
    tip.position.set(x2, y2, z2);
    group.add(tip);
    return group;
}

function makeLegLink(x1, y1, z1, x2, y2, z2, color) {
    var geo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(x1, y1, z1),
        new THREE.Vector3(x2, y2, z2)
    ]);
    var line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: color }));
    line.userData.isOverlay = true;
    return line;
}

function makeGlowMarker(x, y, z, color, radius) {
    var group = new THREE.Group();
    // Core sphere
    var core = new THREE.Mesh(
        new THREE.SphereGeometry(radius, 10, 10),
        new THREE.MeshBasicMaterial({ color: color })
    );
    group.add(core);
    // Outer glow ring
    var ring = new THREE.Mesh(
        new THREE.RingGeometry(radius * 1.3, radius * 2.0, 16),
        new THREE.MeshBasicMaterial({
            color: color, transparent: true, opacity: 0.3,
            side: THREE.DoubleSide, depthWrite: false
        })
    );
    group.add(ring);
    group.position.set(x, y, z);
    return group;
}

function buildOverlay() {
    if (overlayParts.length > 0) return;

    function addOverlay(obj, parent) {
        obj.visible = false;
        obj.traverse(function (c) { c.userData.isOverlay = true; });
        (parent || state.dogGroup).add(obj);
        overlayParts.push(obj);
    }

    // Wireframe body box
    var bodyGeo = new THREE.BoxGeometry(BODY_L * 1.05, BODY_H * 1.3, BODY_W * 1.05);
    var wireframe = new THREE.LineSegments(
        new THREE.EdgesGeometry(bodyGeo),
        new THREE.LineBasicMaterial({ color: 0x00ffff })
    );
    addOverlay(wireframe);

    // IMU coordinate axes (RGB = XYZ)
    var axisLen = BODY_L * 1.2;
    var axes = new THREE.Group();
    axes.add(makeArrow(0, 0, 0, axisLen, 0, 0, 0xff4444));
    axes.add(makeArrow(0, 0, 0, 0, axisLen * 0.7, 0, 0x44ff44));
    axes.add(makeArrow(0, 0, 0, 0, 0, axisLen * 0.7, 0x4488ff));
    addOverlay(axes);

    // Horizon ring — in scene space (not dogGroup)
    var ringGeo = new THREE.RingGeometry(BODY_L * 0.7, BODY_L * 0.75, 32);
    var ring = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({
        color: 0x00ffcc, transparent: true, opacity: 0.35, side: THREE.DoubleSide
    }));
    ring.rotation.x = -Math.PI / 2;
    ring.visible = false;
    ring.userData.isOverlay = true;
    state.scene.add(ring);
    overlayParts.push(ring);
    // Store ref for position update
    ring.userData.isHorizon = true;

    // Per-leg: joint markers + wireframe links
    ["fl", "fr", "rl", "rr"].forEach(function (name) {
        var leg = state.legs[name];
        if (!leg) return;

        var hipMarker = makeGlowMarker(0, 0, 0, 0x00ffcc, 0.012 * S);
        addOverlay(hipMarker, leg.hipPivot);

        var kneeMarker = makeGlowMarker(0, 0, 0, 0xff8800, 0.012 * S);
        addOverlay(kneeMarker, leg.kneePivot);

        var footMarker = makeGlowMarker(0, -LOWER_LEN, 0, 0xff44ff, 0.008 * S);
        addOverlay(footMarker, leg.kneePivot);

        addOverlay(makeLegLink(0, 0, 0, 0, -UPPER_LEN, 0, 0x00ffff), leg.hipPivot);
        addOverlay(makeLegLink(0, 0, 0, 0, -LOWER_LEN, 0, 0x00cccc), leg.kneePivot);

        jointMarkers[name] = { hip: hipMarker, knee: kneeMarker, foot: footMarker };
    });
}

export function updateOverlay() {
    if (!overlayVisible) return;

    // Horizon ring: stays level at dog's world position
    overlayParts.forEach(function (obj) {
        if (obj.userData.isHorizon && state.dogGroup) {
            obj.position.set(
                state.dogGroup.position.x,
                state.dogGroup.position.y,
                state.dogGroup.position.z
            );
        }
    });
}

export function toggleOverlay(show) {
    if (overlayParts.length === 0) buildOverlay();
    overlayVisible = show;

    // Show/hide overlay parts
    overlayParts.forEach(function (obj) { obj.visible = show; });

    // Show/hide normal dog meshes (inverse of overlay)
    if (state.dogGroup) {
        state.dogGroup.traverse(function (child) {
            if (child.isMesh && !child.userData.isOverlay) {
                child.visible = !show;
            }
            if (child.isLine && !child.userData.isOverlay) {
                child.visible = !show;
            }
        });
    }
}
