// Sim noise panel, battery graph, transport switcher
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

// --- Transport Switcher ---
export function setupTransport() {
    var badge = document.getElementById("transport-badge");
    var menu = document.getElementById("transport-menu");

    badge.addEventListener("click", function (e) {
        e.stopPropagation();
        menu.classList.toggle("hidden");
    });
    document.addEventListener("click", function () { menu.classList.add("hidden"); });

    menu.addEventListener("click", function (e) {
        var btn = e.target.closest("[data-transport]");
        if (!btn) return;
        var mode = btn.dataset.transport;
        menu.classList.add("hidden");

        if (mode === "wifi" || mode === "wifi-fw") {
            var host = prompt("MechDog WiFi IP:", "192.168.1.163");
            if (!host) return;
            send({ type: "cmd_transport", mode: mode, wifi_host: host });
        } else if (mode === "wifi-setup") {
            var ssid = prompt("WiFi SSID:");
            if (!ssid) return;
            var pw = prompt("WiFi Password:");
            if (pw === null) return;
            send({ type: "cmd_wifi_setup", ssid: ssid, password: pw });
            badge.textContent = "SETUP..."; badge.className = "transport-badge switching";
            return;
        } else {
            send({ type: "cmd_transport", mode: mode });
        }
        badge.textContent = "..."; badge.className = "transport-badge switching";
    });
}

export function updateTransportUI(msg) {
    var badge = document.getElementById("transport-badge");
    if (msg.ok) {
        document.getElementById("wifi-banner").classList.add("hidden");
    } else {
        badge.textContent = msg.error || "Failed";
        badge.className = "transport-badge sim";
    }
}

export function showWifiBanner(ip, ssid) {
    var banner = document.getElementById("wifi-banner");
    document.getElementById("wifi-banner-text").textContent = "WiFi detected: " + (ssid || ip) + " (" + ip + ")";
    banner.classList.remove("hidden");
    document.getElementById("wifi-banner-switch").onclick = function () {
        send({ type: "cmd_transport", mode: "wifi", wifi_host: ip });
        banner.classList.add("hidden");
        var badge = document.getElementById("transport-badge");
        badge.textContent = "..."; badge.className = "transport-badge switching";
    };
    document.getElementById("wifi-banner-dismiss").onclick = function () { banner.classList.add("hidden"); };
}
