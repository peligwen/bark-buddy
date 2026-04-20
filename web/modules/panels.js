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

export function initOffsetPanel(sendFn) {
    const offsetBtn = document.getElementById('offset-set-btn');
    const pinBtn = document.getElementById('pin-set-btn');
    if (!offsetBtn || !pinBtn) return;

    offsetBtn.addEventListener('click', () => {
        const index = parseInt(document.getElementById('offset-idx').value, 10);
        const offset_us = parseInt(document.getElementById('offset-val').value, 10);
        if (isNaN(index) || isNaN(offset_us)) return;
        sendFn({ type: 'cmd_offset', index, offset_us });
    });

    pinBtn.addEventListener('click', () => {
        const index = parseInt(document.getElementById('pin-idx').value, 10);
        const pin = parseInt(document.getElementById('pin-val').value, 10);
        if (isNaN(index) || isNaN(pin)) return;
        sendFn({ type: 'cmd_servo_pin', index, pin });
    });
}

export function updateServoPins(msg) {
    const el = document.getElementById('servo-pins-display');
    if (!el) return;
    const pins = msg.pins || [];
    if (!pins.length) return;
    el.textContent = 'Pins: ' + pins.map((p, i) => `${i}→${p}`).join('  ');
}

// --- Gait Parameters Panel ---

var GAIT_DEFAULTS = { stride_length: 12, stride_height: 10, frequency: 1.5 };

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

export function initGaitPanel(sendFn) {
    var timer = null;

    function sendGait() {
        var sl = clamp(parseFloat(document.getElementById('gait-stride-length').value), 0, 30);
        var sh = clamp(parseFloat(document.getElementById('gait-stride-height').value), 0, 20);
        var fr = clamp(parseFloat(document.getElementById('gait-frequency').value), 0.5, 3.0);
        sendFn({ type: 'cmd_gait_params', stride_length: sl, stride_height: sh, frequency: fr });
    }

    ['gait-stride-length', 'gait-stride-height', 'gait-frequency'].forEach(function (id) {
        var el = document.getElementById(id);
        if (!el) return;
        var valEl = document.getElementById(id + '-val');
        var decimals = id === 'gait-frequency' ? 1 : 0;
        el.addEventListener('input', function () {
            if (valEl) valEl.textContent = parseFloat(el.value).toFixed(decimals);
            clearTimeout(timer);
            timer = setTimeout(sendGait, 200);
        });
    });

    var resetBtn = document.getElementById('gait-reset-btn');
    if (resetBtn) {
        resetBtn.addEventListener('click', function () {
            document.getElementById('gait-stride-length').value = GAIT_DEFAULTS.stride_length;
            document.getElementById('gait-stride-height').value = GAIT_DEFAULTS.stride_height;
            document.getElementById('gait-frequency').value = GAIT_DEFAULTS.frequency;
            document.getElementById('gait-stride-length-val').textContent = GAIT_DEFAULTS.stride_length;
            document.getElementById('gait-stride-height-val').textContent = GAIT_DEFAULTS.stride_height;
            document.getElementById('gait-frequency-val').textContent = GAIT_DEFAULTS.frequency.toFixed(1);
            sendFn({ type: 'cmd_gait_params',
                     stride_length: GAIT_DEFAULTS.stride_length,
                     stride_height: GAIT_DEFAULTS.stride_height,
                     frequency:     GAIT_DEFAULTS.frequency });
        });
    }
}

// --- Body Transform Panel ---

var XFORM_AXES = [
    { id: 'roll',  min: -15, max:  15, unit: '\u00b0' },
    { id: 'pitch', min: -15, max:  15, unit: '\u00b0' },
    { id: 'yaw',   min: -20, max:  20, unit: '\u00b0' },
    { id: 'x',     min: -25, max:  25, unit: 'mm' },
    { id: 'y',     min: -25, max:  25, unit: 'mm' },
    { id: 'z',     min: -25, max:  25, unit: 'mm' },
];

export function setTransformSliders(payload) {
    XFORM_AXES.forEach(function (ax) {
        var val = (payload[ax.id] !== undefined) ? payload[ax.id] : 0;
        var slider = document.getElementById('xform-' + ax.id);
        var display = document.getElementById('xform-' + ax.id + '-val');
        if (slider) slider.value = val;
        if (display) display.textContent = parseFloat(val).toFixed(1);
    });
}

export function initTransformPanel(sendFn) {
    var timer = null;

    function sendTransform() {
        var msg = { type: 'cmd_transform' };
        XFORM_AXES.forEach(function (ax) {
            var slider = document.getElementById('xform-' + ax.id);
            if (slider) msg[ax.id] = clamp(parseFloat(slider.value), ax.min, ax.max);
        });
        sendFn(msg);
    }

    XFORM_AXES.forEach(function (ax) {
        var slider  = document.getElementById('xform-' + ax.id);
        var display = document.getElementById('xform-' + ax.id + '-val');
        if (!slider) return;
        slider.addEventListener('input', function () {
            if (display) display.textContent = parseFloat(slider.value).toFixed(1);
            clearTimeout(timer);
            timer = setTimeout(sendTransform, 100);
        });
    });

    var resetBtn = document.getElementById('xform-reset-btn');
    if (resetBtn) {
        resetBtn.addEventListener('click', function () {
            setTransformSliders({});
            sendFn({ type: 'cmd_transform' });
        });
    }
}

// --- Servo Nudge Panel ---

export function initServoNudgePanel(sendFn) {
    var pulses = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500];

    var minusBtn = document.getElementById('servo-nudge-minus');
    var plusBtn  = document.getElementById('servo-nudge-plus');
    if (!minusBtn || !plusBtn) return;

    function nudge(sign) {
        var idx  = parseInt(document.getElementById('servo-nudge-index').value, 10);
        var step = parseInt(document.getElementById('servo-nudge-step').value, 10);
        var next = clamp(pulses[idx] + sign * step, 500, 2500);
        pulses[idx] = next;
        var readout = document.getElementById('servo-nudge-readout-' + idx);
        if (readout) readout.textContent = next;
        sendFn({ type: 'cmd_servo', index: idx, pulse_us: next });
    }

    minusBtn.addEventListener('click', function () { nudge(-1); });
    plusBtn.addEventListener('click',  function () { nudge(+1); });
}
