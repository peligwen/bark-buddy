// Battery graph, sim noise panel, OTA panel
import { send } from './ws.js';

// --- Battery Graph ---
var battHistory = [];
var BATT_MAX_POINTS = 300;

export function setupBatteryGraph() {
    document.getElementById("batt-stat").addEventListener("click", function (e) {
        e.stopPropagation();
        var popup = document.getElementById("batt-popup");
        popup.classList.toggle("hidden");
        if (!popup.classList.contains("hidden")) drawBattGraph();
    });
    document.getElementById("batt-popup-close").addEventListener("click", function () {
        document.getElementById("batt-popup").classList.add("hidden");
    });
}

export function recordBattery(pct) {
    var now = Date.now() / 1000;
    battHistory.push({ time: now, pct: pct });
    if (battHistory.length > BATT_MAX_POINTS) battHistory.shift();
    if (!document.getElementById("batt-popup").classList.contains("hidden")) drawBattGraph();
}

function drawBattGraph() {
    var canvas = document.getElementById("batt-canvas");
    var rect = canvas.parentElement.getBoundingClientRect();
    canvas.width = Math.floor(rect.width); canvas.height = 120;
    var ctx = canvas.getContext("2d");
    var w = canvas.width - 16, h = canvas.height - 16, ox = 8, oy = 8;

    ctx.fillStyle = "#151820"; ctx.fillRect(0, 0, canvas.width, canvas.height);

    if (battHistory.length < 2) {
        ctx.fillStyle = "#4b5160"; ctx.font = "11px monospace";
        ctx.fillText("Waiting for data...", ox + 4, oy + h / 2); return;
    }

    var tMin = battHistory[0].time, tMax = battHistory[battHistory.length - 1].time;
    var tRange = Math.max(tMax - tMin, 1);

    ctx.strokeStyle = "#252a35"; ctx.lineWidth = 1;
    for (var g = 25; g <= 75; g += 25) {
        var gy = oy + h - (g / 100) * h;
        ctx.beginPath(); ctx.moveTo(ox, gy); ctx.lineTo(ox + w, gy); ctx.stroke();
    }

    ctx.beginPath(); ctx.lineWidth = 2;
    for (var i = 0; i < battHistory.length; i++) {
        var pt = battHistory[i];
        var x = ox + ((pt.time - tMin) / tRange) * w;
        var y = oy + h - (Math.max(0, Math.min(100, pt.pct)) / 100) * h;
        ctx.strokeStyle = pt.pct < 20 ? "#ef4444" : pt.pct < 40 ? "#f59e0b" : "#22c55e";
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();

    var last = battHistory[battHistory.length - 1];
    ctx.fillStyle = "#e0e0e0"; ctx.font = "11px monospace";
    ctx.fillText(last.pct + "%", ox + w - 30, oy + 12);
    var mins = Math.floor(tRange / 60), secs = Math.floor(tRange % 60);
    ctx.fillStyle = "#4b5160"; ctx.font = "9px monospace";
    ctx.fillText(mins > 0 ? mins + "m " + secs + "s" : secs + "s", ox + 2, oy + h - 2);
}

// --- Sim Noise Panel ---
var noiseValueIds = {
    "packet_drop_pct": "val-drop", "latency_base_ms": "val-lat",
    "latency_jitter_ms": "val-jit", "sonar_noise_mm": "val-snoise",
    "sonar_outlier_pct": "val-out", "imu_drift_dps": "val-drift",
};

export function setupNoisePanel() {
    var sliders = document.querySelectorAll("#sim-panel input[data-noise]");
    var debounceTimer = null;

    function sendNoiseParams() {
        var params = {};
        sliders.forEach(function (s) { params[s.dataset.noise] = parseFloat(s.value); });
        send({ type: "cmd_sim_noise", params: params });
    }

    sliders.forEach(function (slider) {
        slider.addEventListener("input", function () {
            var valEl = document.getElementById(noiseValueIds[slider.dataset.noise]);
            if (valEl) valEl.textContent = slider.value;
            clearTimeout(debounceTimer);
            debounceTimer = setTimeout(sendNoiseParams, 150);
        });
    });
}

export function syncNoiseSliders(params) {
    for (var key in params) {
        var slider = document.querySelector('#sim-panel input[data-noise="' + key + '"]');
        if (slider) {
            slider.value = params[key];
            var valEl = document.getElementById(noiseValueIds[key]);
            if (valEl) valEl.textContent = Math.round(params[key] * 10) / 10;
        }
    }
}

// --- OTA Firmware Update Panel ---

export function setupOtaPanel() {
    var fwBadge = document.getElementById('fw-badge');
    var modal   = document.getElementById('ota-modal');
    var startBtn = document.getElementById('ota-start');
    var cancelBtn = document.getElementById('ota-cancel');

    fwBadge.addEventListener('click', async function () {
        if (!fwBadge.classList.contains('outdated')) return;
        try {
            var resp = await fetch('/api/firmware/status');
            var data = await resp.json();
            document.getElementById('ota-current').textContent   = data.current_version || '--';
            document.getElementById('ota-available').textContent = data.available_version || '--';
            startBtn.disabled = !data.can_ota;
            startBtn.title = data.can_ota ? '' : 'OTA requires WiFi connection';
        } catch (e) {
            document.getElementById('ota-current').textContent   = '--';
            document.getElementById('ota-available').textContent = '--';
        }
        document.getElementById('ota-error').classList.add('hidden');
        document.getElementById('ota-progress').classList.add('hidden');
        modal.classList.remove('hidden');
    });

    cancelBtn.addEventListener('click', function () { modal.classList.add('hidden'); });

    startBtn.addEventListener('click', async function () {
        startBtn.disabled = true;
        cancelBtn.disabled = true;
        document.getElementById('ota-progress').classList.remove('hidden');
        document.getElementById('ota-error').classList.add('hidden');
        setOtaProgress('building', 10);
        try {
            var resp = await fetch('/api/firmware/update', { method: 'POST' });
            var data = await resp.json();
            if (!data.ok) {
                showOtaError(data.error || 'Update failed');
                cancelBtn.disabled = false;
            }
            // Further progress comes via WebSocket ota_status messages
        } catch (e) {
            showOtaError('Network error: ' + e.message);
            cancelBtn.disabled = false;
        }
    });
}

export function updateOtaStatus(status, error) {
    // Auto-show modal if OTA is in progress (handles page refresh recovery)
    if (status === 'downloading' || status === 'flashing') {
        var modal = document.getElementById('ota-modal');
        if (modal) modal.classList.remove('hidden');
    }
    var fill = document.getElementById('ota-progress-fill');
    var text = document.getElementById('ota-status-text');
    if (!fill || !text) return;
    switch (status) {
        case 'downloading':
            text.textContent = 'Downloading firmware...'; fill.style.width = '40%'; break;
        case 'flashing':
            text.textContent = 'Flashing...';             fill.style.width = '75%'; break;
        case 'complete':
            text.textContent = 'Complete! Rebooting...';  fill.style.width = '100%'; break;
        case 'failed':
            showOtaError(error || 'Flash failed');         break;
    }
}

function setOtaProgress(label, pct) {
    var fill = document.getElementById('ota-progress-fill');
    var text = document.getElementById('ota-status-text');
    if (fill) fill.style.width = pct + '%';
    if (text) text.textContent = label;
}

function showOtaError(msg) {
    var el = document.getElementById('ota-error');
    if (el) { el.textContent = msg; el.classList.remove('hidden'); }
    var start = document.getElementById('ota-start');
    if (start) start.disabled = false;
}
