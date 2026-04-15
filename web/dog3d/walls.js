// Wall mesh rendering from point cloud chains and line segments.
import { state, S } from './state.js';

var wallMeshes = [];
var wallTexture = null;

function createWallTexture() {
    var size = 256;
    var c = document.createElement("canvas");
    c.width = size; c.height = size;
    var ctx = c.getContext("2d");

    ctx.fillStyle = "#8a8a86";
    ctx.fillRect(0, 0, size, size);

    var imgData = ctx.getImageData(0, 0, size, size);
    var d = imgData.data;
    for (var i = 0; i < d.length; i += 4) {
        var n = (Math.random() - 0.5) * 18;
        d[i] += n; d[i + 1] += n; d[i + 2] += n;
    }
    ctx.putImageData(imgData, 0, 0);

    ctx.strokeStyle = "rgba(50, 50, 48, 0.6)";
    ctx.lineWidth = 2;
    for (var y = 64; y < size; y += 64) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(size, y); ctx.stroke();
        ctx.strokeStyle = "rgba(160, 160, 155, 0.3)";
        ctx.beginPath(); ctx.moveTo(0, y + 2); ctx.lineTo(size, y + 2); ctx.stroke();
        ctx.strokeStyle = "rgba(50, 50, 48, 0.6)";
    }
    for (var x = 128; x < size; x += 128) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, size); ctx.stroke();
        ctx.strokeStyle = "rgba(160, 160, 155, 0.3)";
        ctx.beginPath(); ctx.moveTo(x + 2, 0); ctx.lineTo(x + 2, size); ctx.stroke();
        ctx.strokeStyle = "rgba(50, 50, 48, 0.6)";
    }
    ctx.strokeStyle = "rgba(200, 180, 60, 0.2)"; ctx.lineWidth = 1;
    for (var ty = 32; ty < size; ty += 32) {
        ctx.beginPath(); ctx.moveTo(0, ty); ctx.lineTo(8, ty); ctx.stroke();
    }
    ctx.fillStyle = "rgba(200, 160, 20, 0.15)";
    ctx.fillRect(0, size - 12, size, 12);

    var tex = new THREE.CanvasTexture(c);
    tex.wrapS = THREE.RepeatWrapping; tex.wrapT = THREE.RepeatWrapping;
    return tex;
}

export function clearWalls() {
    wallMeshes.forEach(function (m) {
        if (m.geometry) m.geometry.dispose();
        state.scene.remove(m);
    });
    wallMeshes = [];
}

// Build wall meshes from vertex chains (new approach)
export function buildWallsFromChains(chains) {
    clearWalls();
    if (!chains || chains.length < 1) return;
    if (!wallTexture) wallTexture = createWallTexture();

    var wallMat = new THREE.MeshStandardMaterial({
        map: wallTexture, roughness: 0.85, metalness: 0.05, side: THREE.DoubleSide,
    });
    var edgeMat = new THREE.LineBasicMaterial({
        color: 0x555555, transparent: true, opacity: 0.4,
    });

    for (var c = 0; c < chains.length; c++) {
        var chain = chains[c];
        var verts = chain.vertices;
        if (!verts || verts.length < 2) continue;

        var height = (chain.height || 0.2) * S;

        // Build triangle strip: for each vertex, create top + bottom points
        // Each consecutive pair of vertices forms a quad (2 triangles)
        var positions = [];
        var uvs = [];
        var indices = [];
        var accumLen = 0;

        for (var i = 0; i < verts.length; i++) {
            var sx = verts[i][0] * S;
            var sz = verts[i][1] * S;

            // Bottom vertex
            positions.push(sx, 0, sz);
            // Top vertex
            positions.push(sx, height, sz);

            // UV: x = accumulated length along chain, y = 0 (bottom) or 1 (top)
            if (i > 0) {
                var dx = verts[i][0] - verts[i-1][0];
                var dz = verts[i][1] - verts[i-1][1];
                accumLen += Math.sqrt(dx * dx + dz * dz);
            }
            var u = accumLen * 2; // texture repeats every 0.5m
            uvs.push(u, 0);  // bottom
            uvs.push(u, 1);  // top

            // Indices: connect to previous vertex pair as a quad
            if (i > 0) {
                var bl = (i - 1) * 2;     // bottom-left
                var tl = (i - 1) * 2 + 1; // top-left
                var br = i * 2;            // bottom-right
                var tr = i * 2 + 1;        // top-right

                indices.push(bl, br, tl);  // triangle 1
                indices.push(tl, br, tr);  // triangle 2
            }
        }

        var geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        geo.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
        geo.setIndex(indices);
        geo.computeVertexNormals();

        var mesh = new THREE.Mesh(geo, wallMat);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        state.scene.add(mesh);
        wallMeshes.push(mesh);

        // Edge wireframe for visual depth
        var edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), edgeMat);
        state.scene.add(edges);
        wallMeshes.push(edges);
    }
}
