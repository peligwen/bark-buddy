// 2D scan map canvas rendering
export var dogMapState = { x: 0, y: 0, heading: 0, motion: "stop" };
export var mapPoints = [];
export var mapScans = [];
export var mapWalls = [];
export var mapChains = [];
export var mapBounds = { min_x: -2, max_x: 2, min_y: -2, max_y: 2 };

export function addScanPoint(msg) {
    mapPoints.push({ x: msg.x, y: msg.y, distance_mm: msg.distance_mm,
                     confidence: msg.confidence != null ? msg.confidence : 0.5 });
    updateMapBoundsFromPoint(msg.x, msg.y);
    drawMap();
    document.getElementById("scan-progress-fill").style.width = (msg.progress || 0) + "%";
    document.getElementById("scan-progress-text").textContent = (msg.progress || 0) + "%";
    document.getElementById("map-points").textContent = mapPoints.length + " pts";
}

export function renderFullMap(data, Dog3D) {
    mapPoints = data.points || [];
    mapScans = data.scans || [];
    mapWalls = data.walls || [];
    mapChains = data.chains || [];
    mapBounds = data.bounds || mapBounds;
    document.getElementById("map-points").textContent = (data.point_count || 0) + " pts";
    document.getElementById("map-scans").textContent = (data.scan_count || 0) + " scans";
    drawMap();
    Dog3D.setMapData(data);
}

function updateMapBoundsFromPoint(x, y) {
    var margin = 0.2;
    if (x - margin < mapBounds.min_x) mapBounds.min_x = x - margin;
    if (x + margin > mapBounds.max_x) mapBounds.max_x = x + margin;
    if (y - margin < mapBounds.min_y) mapBounds.min_y = y - margin;
    if (y + margin > mapBounds.max_y) mapBounds.max_y = y + margin;
}

export function drawMap() {
    var canvas = document.getElementById("map-canvas");
    var rect = canvas.parentElement.getBoundingClientRect();
    var info = document.getElementById("map-info");
    var progress = document.getElementById("scan-progress");
    var usedH = info.offsetHeight + (progress.classList.contains("hidden") ? 0 : progress.offsetHeight);
    var cw = Math.floor(rect.width);
    var ch = Math.floor(rect.height - usedH);
    if (cw > 0 && ch > 0 && (canvas.width !== cw || canvas.height !== ch)) {
        canvas.width = cw; canvas.height = ch;
    }

    var ctx = canvas.getContext("2d");
    var w = canvas.width, h = canvas.height;

    ctx.fillStyle = "#151820";
    ctx.fillRect(0, 0, w, h);

    var rangeX = mapBounds.max_x - mapBounds.min_x;
    var rangeY = mapBounds.max_y - mapBounds.min_y;
    var range = Math.max(rangeX, rangeY, 0.5);
    var centerX = (mapBounds.min_x + mapBounds.max_x) / 2;
    var centerY = (mapBounds.min_y + mapBounds.max_y) / 2;
    var scale = (Math.min(w, h) - 40) / range;

    function toCanvasX(x) { return w / 2 + (x - centerX) * scale; }
    function toCanvasY(y) { return h / 2 + (y - centerY) * scale; }

    // Grid
    ctx.strokeStyle = "#252a35"; ctx.lineWidth = 1;
    var gridStep = gridStepForRange(range);
    var gx = Math.floor(mapBounds.min_x / gridStep) * gridStep;
    while (gx <= mapBounds.max_x) { var cx = toCanvasX(gx); ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke(); gx += gridStep; }
    var gy = Math.floor(mapBounds.min_y / gridStep) * gridStep;
    while (gy <= mapBounds.max_y) { var cy = toCanvasY(gy); ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke(); gy += gridStep; }

    // Origin cross
    ctx.strokeStyle = "#3a3f4a"; ctx.lineWidth = 1;
    var ox = toCanvasX(0), oy = toCanvasY(0);
    ctx.beginPath(); ctx.moveTo(ox - 8, oy); ctx.lineTo(ox + 8, oy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(ox, oy - 8); ctx.lineTo(ox, oy + 8); ctx.stroke();

    // Scan origins
    ctx.fillStyle = "rgba(212, 136, 14, 0.5)";
    for (var s = 0; s < mapScans.length; s++) {
        var sc = mapScans[s]; var sx = toCanvasX(sc.x); var sy = toCanvasY(sc.y);
        ctx.beginPath(); ctx.arc(sx, sy, 3, 0, Math.PI * 2); ctx.fill();
    }

    // Wall chains (polylines through vertex chains)
    if (mapChains && mapChains.length > 0) {
        ctx.strokeStyle = "#6366f1"; ctx.lineWidth = 2; ctx.lineCap = "round"; ctx.lineJoin = "round";
        for (var ci = 0; ci < mapChains.length; ci++) {
            var chain = mapChains[ci];
            var verts = chain.vertices;
            if (!verts || verts.length < 2) continue;
            ctx.globalAlpha = 0.3 + 0.7 * (chain.confidence || 0.5);
            ctx.beginPath();
            ctx.moveTo(toCanvasX(verts[0][0]), toCanvasY(verts[0][1]));
            for (var vi = 1; vi < verts.length; vi++) {
                ctx.lineTo(toCanvasX(verts[vi][0]), toCanvasY(verts[vi][1]));
            }
            ctx.stroke();
        }
        ctx.globalAlpha = 1.0;
    } else if (mapWalls && mapWalls.length > 0) {
        // Legacy fallback: wall segments
        ctx.strokeStyle = "#6366f1"; ctx.lineWidth = 3; ctx.lineCap = "round";
        for (var wi = 0; wi < mapWalls.length; wi++) {
            var wall = mapWalls[wi];
            ctx.globalAlpha = 0.3 + 0.7 * (wall.confidence || 0.5);
            ctx.beginPath();
            ctx.moveTo(toCanvasX(wall.x1), toCanvasY(wall.y1));
            ctx.lineTo(toCanvasX(wall.x2), toCanvasY(wall.y2));
            ctx.stroke();
        }
        ctx.globalAlpha = 1.0;
    }

    // Obstacle points — colored by confidence
    for (var i = 0; i < mapPoints.length; i++) {
        var p = mapPoints[i]; var px = toCanvasX(p.x); var py = toCanvasY(p.y);
        var conf = p.confidence != null ? p.confidence : 1.0;
        if (conf > 0.6) ctx.fillStyle = "#22c55e";
        else if (conf > 0.3) ctx.fillStyle = "#f59e0b";
        else ctx.fillStyle = "#ef4444";
        ctx.globalAlpha = 0.4 + 0.6 * conf;
        ctx.beginPath(); ctx.arc(px, py, 3, 0, Math.PI * 2); ctx.fill();
    }
    ctx.globalAlpha = 1.0;

    // Dog position triangle
    var dx = toCanvasX(dogMapState.x); var dy = toCanvasY(dogMapState.y);
    var headingRad = dogMapState.heading * (Math.PI / 180);
    var triSize = 8;
    ctx.save(); ctx.translate(dx, dy); ctx.rotate(headingRad);
    ctx.beginPath(); ctx.moveTo(triSize, 0); ctx.lineTo(-triSize * 0.6, -triSize * 0.5); ctx.lineTo(-triSize * 0.6, triSize * 0.5); ctx.closePath();
    ctx.fillStyle = "#d4880e"; ctx.fill(); ctx.strokeStyle = "#fff"; ctx.lineWidth = 1; ctx.stroke();
    ctx.restore();

    // Scale label
    ctx.fillStyle = "#4b5160"; ctx.font = "10px monospace";
    ctx.fillText(gridStep.toFixed(1) + "m", 6, h - 6);
}

function gridStepForRange(range) {
    if (range < 1) return 0.2;
    if (range < 3) return 0.5;
    if (range < 6) return 1.0;
    return 2.0;
}

export function setupScan() {
    var btnStart = document.getElementById("btn-scan-start");
    var btnStop = document.getElementById("btn-scan-stop");
    var btnClear = document.getElementById("btn-map-clear");
    var sendFn = null;

    return {
        init: function(send) {
            sendFn = send;
            btnStart.addEventListener("click", function () { sendFn({ type: "cmd_scan", action: "start" }); });
            btnStop.addEventListener("click", function () { sendFn({ type: "cmd_scan", action: "stop" }); });
            btnClear.addEventListener("click", function () { sendFn({ type: "cmd_map", action: "clear" }); });
            drawMap();
        },
        setScanRunning: function(running) {
            btnStart.disabled = running;
            btnStop.disabled = !running;
            var progress = document.getElementById("scan-progress");
            if (running) progress.classList.remove("hidden");
            else progress.classList.add("hidden");
        }
    };
}
