// Bark-Buddy Web UI — ES module entry point
import Dog3D from './dog3d/index.js';
import { connect, send, setMessageHandler } from './modules/ws.js';
import { setupDpad, setupKeyboard, setupActions, setCanControl } from './modules/controls.js';
import { dogMapState, addScanPoint, renderFullMap, drawMap, setupScan } from './modules/map.js';
import { setupBatteryGraph, recordBattery, setupNoisePanel, syncNoiseSliders,
         setupTransport, updateTransportUI, showWifiBanner, handleWifiSetupResult } from './modules/panels.js';

// --- State ---
var hasLock = false;
var lockHolder = null;
var operatorName = new URLSearchParams(location.search).get("name") || "Operator";

function canControl() {
    if (window._appVersion && window._serverVersion && window._appVersion !== window._serverVersion) {
        location.reload(); return false;
    }
    return lockHolder === null || hasLock;
}

setCanControl(canControl);

// --- Telemetry helpers ---
function batteryPercent(mv) {
    var pct = Math.round(((mv - 6000) / 2400) * 100);
    return Math.max(0, Math.min(100, pct));
}

function updateMotionIndicator(motion) {
    var el = document.getElementById("motion-state");
    if (!motion || motion === "stop") { el.textContent = ""; el.className = "motion-indicator"; }
    else { el.textContent = motion.toUpperCase(); el.className = "motion-indicator active"; }
}

function showFallAlert(fallen) {
    var el = document.getElementById("fall-alert");
    if (fallen) el.classList.remove("hidden"); else el.classList.add("hidden");
}

function updateUltrasonic(mm) {
    Dog3D.updateUltrasonic(mm);
    var el = document.getElementById("ultra-val");
    el.textContent = mm + "mm";
    if (mm < 100) el.className = "status-value danger-text";
    else if (mm < 300) el.className = "status-value warn-text";
    else el.className = "status-value";
}

// --- Gauge (hidden elements for compat) ---
function updateGauge(name, value) {
    var val = document.getElementById(name + "-val");
    val.textContent = value.toFixed(1) + "\u00B0";
}

// --- Status update ---
var actionsCtrl = null;

function updateStatus(msg) {
    if (msg.battery_mv != null) {
        var pct = batteryPercent(msg.battery_mv);
        document.getElementById("battery-val").textContent = pct + "%";
        recordBattery(pct);
    }
    if (msg.mode != null) {
        document.getElementById("mode-val").textContent = msg.mode;
        scanCtrl.setScanRunning(msg.mode === "scan");
    }
    if (msg.balance != null && actionsCtrl) actionsCtrl.setBalanceState(msg.balance);
    if (msg.fallen != null) showFallAlert(msg.fallen);
    if (msg.transport != null) {
        var badge = document.getElementById("transport-badge");
        var isSim = msg.transport === "sim" || msg.transport === "sim+";
        badge.textContent = isSim ? msg.transport.toUpperCase() : msg.transport;
        badge.className = "transport-badge " + (isSim ? "sim" : "live");
        var simPanel = document.getElementById("sim-panel");
        if (isSim) simPanel.classList.remove("hidden");
        else simPanel.classList.add("hidden");
    }
    if (msg.wifi_available && msg.wifi_ip) showWifiBanner(msg.wifi_ip, msg.wifi_ssid);
    if (msg.noise_params) syncNoiseSliders(msg.noise_params);
    if (msg.scan_progress != null) {
        document.getElementById("scan-progress-fill").style.width = msg.scan_progress + "%";
        document.getElementById("scan-progress-text").textContent = msg.scan_progress + "%";
    }
}

// --- Lock UI ---
function updateLockUI(msg) {
    hasLock = msg.is_you;
    lockHolder = msg.holder;
    var btn = document.getElementById("btn-lock");
    var text = document.getElementById("lock-text");
    var header = document.querySelector("header");
    if (msg.is_you) {
        btn.textContent = "Release"; btn.classList.add("locked");
        text.textContent = "You have control"; header.className = "control-self";
    } else if (msg.locked) {
        btn.textContent = "Request"; btn.classList.remove("locked");
        text.textContent = msg.holder + " has control"; header.className = "control-other";
    } else {
        btn.textContent = "Take Control"; btn.classList.remove("locked");
        text.textContent = ""; header.className = "";
    }
}

// --- Message handler ---
function handleMessage(msg) {
    if (msg.type === "telem_imu") {
        updateGauge("pitch", msg.pitch);
        updateGauge("roll", msg.roll);
        Dog3D.updateIMU(msg);
    } else if (msg.type === "telem_odometry") {
        Dog3D.updateOdometry(msg);
        dogMapState.x = msg.x || 0;
        dogMapState.y = msg.y || 0;
        dogMapState.heading = msg.heading || 0;
        dogMapState.motion = msg.motion || "stop";
        updateMotionIndicator(msg.motion);
        if (msg.heading != null) {
            var h = Math.round(msg.heading) % 360;
            if (h < 0) h += 360;
            document.getElementById("heading-val").textContent = h + "\u00B0";
        }
        drawMap();
    } else if (msg.type === "telem_status") {
        updateStatus(msg);
    } else if (msg.type === "balance_state") {
        if (actionsCtrl) actionsCtrl.setBalanceState(msg.enabled);
    } else if (msg.type === "telem_ultrasonic") {
        updateUltrasonic(msg.distance_mm);
    } else if (msg.type === "event_fall") {
        showFallAlert(true); Dog3D.setFallen(true);
    } else if (msg.type === "event_recovered") {
        showFallAlert(false); Dog3D.setFallen(false);
    } else if (msg.type === "scan_point") {
        addScanPoint(msg);
    } else if (msg.type === "scan_complete") {
        scanCtrl.setScanRunning(false);
    } else if (msg.type === "map_data") {
        renderFullMap(msg, Dog3D);
    } else if (msg.type === "lock_status") {
        updateLockUI(msg);
    } else if (msg.type === "lock_challenge") {
        if (confirm(msg.challenger + " wants control. Yield?")) {
            send({ type: "cmd_lock_yield" });
        }
    } else if (msg.type === "reset") {
        Dog3D.reset(); scanCtrl.setScanRunning(false);
    } else if (msg.type === "transport_result") {
        updateTransportUI(msg);
    } else if (msg.type === "wifi_setup_result") {
        handleWifiSetupResult(msg);
        var badge = document.getElementById("transport-badge");
        if (msg.ok) {
            badge.className = "transport-badge live";
            badge.textContent = "wifi:" + msg.ip;
            showWifiBanner(msg.ip, "");
        }
    } else if (msg.type === "version") {
        if (window._appVersion && msg.hash !== window._appVersion) location.reload();
        window._appVersion = msg.hash;
        window._serverVersion = msg.hash;
    }
}

// --- Patrol ---
function setupPatrol() {
    var btnDemo = document.getElementById("btn-patrol-demo");
    var btnStop = document.getElementById("btn-patrol-stop");
    btnDemo.addEventListener("click", function () {
        send({ type: "cmd_patrol", action: "start", waypoints: [
            { x: 0.5, y: 0, heading: 0 }, { x: 0.5, y: 0.5, heading: 90 },
            { x: 0, y: 0.5, heading: 180 }, { x: 0, y: 0, heading: 270 },
        ]});
    });
    btnStop.addEventListener("click", function () { send({ type: "cmd_patrol", action: "stop" }); });
}

function setupLock() {
    document.getElementById("btn-lock").addEventListener("click", function () {
        if (hasLock) send({ type: "cmd_unlock" });
        else send({ type: "cmd_lock", name: operatorName });
    });
}

function setupReset() {
    document.getElementById("btn-reset").addEventListener("click", function () { send({ type: "cmd_reset" }); });
    document.getElementById("btn-restart").addEventListener("click", function () {
        if (confirm("Restart the server?")) send({ type: "cmd_restart_server" });
    });
}

// --- Init ---
Dog3D.init("dog-3d-container");
setMessageHandler(handleMessage);

actionsCtrl = setupActions(Dog3D);
setupDpad();
setupKeyboard();
setupPatrol();
var scanCtrl = setupScan();
scanCtrl.init(send);
setupLock();
setupReset();
setupTransport();
setupNoisePanel();
setupBatteryGraph();

// Kinematics overlay toggle (button + K key)
var overlayOn = false;
function toggleKinematics() {
    overlayOn = !overlayOn;
    Dog3D.toggleOverlay(overlayOn);
    document.getElementById("btn-overlay").classList.toggle("active", overlayOn);
}
document.getElementById("btn-overlay").addEventListener("click", function (e) {
    e.stopPropagation(); toggleKinematics();
});
document.addEventListener("keydown", function (e) {
    if (e.key === "k" || e.key === "K") toggleKinematics();
});

// On WS open, identify and request map
window._onWsOpen = function () {
    send({ type: "cmd_identify", name: operatorName });
    send({ type: "cmd_map", action: "get" });
};

connect();
