// Scan-point wall visualization.
import { state, S } from './state.js';

var wallMeshes = [];
var wallTexture = null;
var WALL_HEIGHT = 2.0; // scene units (~1.7x dog height)
var WALL_THICKNESS = 0.08 * S;

function createWallTexture() {
    // Procedural concrete panel texture — robotics test facility style
    var size = 256;
    var c = document.createElement("canvas");
    c.width = size; c.height = size;
    var ctx = c.getContext("2d");

    // Base concrete gray with slight warm tint
    ctx.fillStyle = "#8a8a86";
    ctx.fillRect(0, 0, size, size);

    // Noise grain
    var imgData = ctx.getImageData(0, 0, size, size);
    var d = imgData.data;
    for (var i = 0; i < d.length; i += 4) {
        var n = (Math.random() - 0.5) * 18;
        d[i] += n; d[i + 1] += n; d[i + 2] += n;
    }
    ctx.putImageData(imgData, 0, 0);

    // Horizontal panel seams (every 64px = ~0.3m panels)
    ctx.strokeStyle = "rgba(50, 50, 48, 0.6)";
    ctx.lineWidth = 2;
    for (var y = 64; y < size; y += 64) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(size, y);
        ctx.stroke();
        // Highlight line below seam
        ctx.strokeStyle = "rgba(160, 160, 155, 0.3)";
        ctx.beginPath();
        ctx.moveTo(0, y + 2);
        ctx.lineTo(size, y + 2);
        ctx.stroke();
        ctx.strokeStyle = "rgba(50, 50, 48, 0.6)";
    }

    // Vertical panel seams (every 128px = ~0.6m panels)
    for (var x = 128; x < size; x += 128) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, size);
        ctx.stroke();
        ctx.strokeStyle = "rgba(160, 160, 155, 0.3)";
        ctx.beginPath();
        ctx.moveTo(x + 2, 0);
        ctx.lineTo(x + 2, size);
        ctx.stroke();
        ctx.strokeStyle = "rgba(50, 50, 48, 0.6)";
    }

    // Subtle measurement markings — small ticks every 32px
    ctx.strokeStyle = "rgba(200, 180, 60, 0.2)";
    ctx.lineWidth = 1;
    for (var ty = 32; ty < size; ty += 32) {
        ctx.beginPath();
        ctx.moveTo(0, ty);
        ctx.lineTo(8, ty);
        ctx.stroke();
    }

    // Bottom safety stripe
    var stripeH = 12;
    ctx.fillStyle = "rgba(200, 160, 20, 0.15)";
    ctx.fillRect(0, size - stripeH, size, stripeH);

    var tex = new THREE.CanvasTexture(c);
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(1, 1);
    return tex;
}

export function clearWalls() {
    wallMeshes.forEach(function (m) {
        if (m.geometry) m.geometry.dispose();
        state.scene.remove(m);
    });
    wallMeshes = [];
}

export function buildWallsFromSegments(walls) {
    clearWalls();
    if (!walls || walls.length < 1) return;
    if (!wallTexture) wallTexture = createWallTexture();

    var wallMat = new THREE.MeshStandardMaterial({
        map: wallTexture,
        roughness: 0.85,
        metalness: 0.05,
        side: THREE.DoubleSide,
    });

    var edgeMat = new THREE.LineBasicMaterial({
        color: 0x555555, transparent: true, opacity: 0.4,
    });

    for (var i = 0; i < walls.length; i++) {
        var w = walls[i];
        // Convert sim coords to scene: sim (x,y) -> scene (x*S, y*S as z)
        var sx1 = w.x1 * S, sz1 = w.y1 * S;
        var sx2 = w.x2 * S, sz2 = w.y2 * S;
        var dx = sx2 - sx1, dz = sz2 - sz1;
        var len = Math.sqrt(dx * dx + dz * dz);
        if (len < 0.01) continue;

        var cx = (sx1 + sx2) / 2;
        var cz = (sz1 + sz2) / 2;
        var angle = Math.atan2(dz, dx);
        var height = (w.height || 0.2) * S;

        var geo = new THREE.BoxGeometry(len, height, WALL_THICKNESS);
        var mesh = new THREE.Mesh(geo, wallMat);
        mesh.position.set(cx, height / 2, cz);
        mesh.rotation.y = -angle;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        state.scene.add(mesh);
        wallMeshes.push(mesh);

        var edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), edgeMat);
        edges.position.copy(mesh.position);
        edges.rotation.copy(mesh.rotation);
        state.scene.add(edges);
        wallMeshes.push(edges);
    }
}
