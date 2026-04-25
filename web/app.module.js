// Bark-Buddy Web UI — ES module entry point
import Dog3D from './dog3d/index.js';
import { connect, send, setMessageHandler } from './modules/ws.js';
import { setupDpad, setupKeyboard, setupActions, setCanControl, setEngaged } from './modules/controls.js';
import { setupBatteryGraph, recordBattery,
         setupOtaPanel, updateOtaStatus,
         initOffsetPanel, updateServoPins,
         initGaitPanel, initTransformPanel,
         initServoNudgePanel } from './modules/panels.js';
import { diagInit, diagHandleTelem } from './modules/diag.js';

// --- State ---
var hasLock = false;
var lockHolder = null;
var operatorName = new URLSearchParams(location.search).get("name") || "Operator";

function canControl() {
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
    if (fallen) { el.textContent = "FALL DETECTED"; el.classList.remove("hidden"); } else el.classList.add("hidden");
}

function updateUltrasonic(mm) {
    var el = document.getElementById("ultra-val");
    el.textContent = mm + "mm";
    if (mm < 100) el.className = "status-value danger-text";
    else if (mm < 300) el.className = "status-value warn-text";
    else el.className = "status-value";
}

// IMU pitch/roll text display
function updateGauge(name, value) {
    var val = document.getElementById(name + "-val");
    val.textContent = value.toFixed(1) + "\u00B0";
}

// --- Status update ---
var actionsCtrl = null;

function updateStatus(msg) {
    if (msg.battery_mv != null) {
        var present = msg.battery_present !== false;
        if (present) {
            var pct = batteryPercent(msg.battery_mv);
            document.getElementById("battery-val").textContent = pct + "%";
            recordBattery(pct);
        } else {
            document.getElementById("battery-val").textContent = "USB";
        }
    }
    if (msg.mode != null) {
        document.getElementById("mode-val").textContent = msg.mode;
    }
    if (msg.balance != null && actionsCtrl) actionsCtrl.setBalanceState(msg.balance);
    if (msg.engaged != null || msg.ramping != null || msg.battery_cutoff != null || msg.battery_present != null) {
        var engaged = msg.engaged === true;
        var ramping = msg.ramping === true;
        var cutoff = msg.battery_cutoff === true;
        var absent = msg.battery_present === false;
        setEngaged(engaged, ramping);
        updateEngageToggle(engaged, ramping, cutoff, absent);
    }
    if (msg.fallen != null) showFallAlert(msg.fallen);
    if (msg.transport != null) {
        var badge = document.getElementById("transport-badge");
        var isMock = msg.transport === "mock";
        badge.textContent = isMock ? "MOCK" : msg.transport;
        badge.className = "transport-badge " + (isMock ? "mock" : "live");
    }
    // Update firmware version badge
    var fwBadge = document.getElementById('fw-badge');
    if (fwBadge && msg.fw_version != null) {
        fwBadge.textContent = 'FW ' + (msg.fw_version || '--');
        if (msg.fw_version && msg.available_fw_version &&
            msg.fw_version !== msg.available_fw_version) {
            fwBadge.className = 'fw-badge outdated';
            fwBadge.title = 'Update available: ' + msg.available_fw_version;
        } else if (msg.fw_version) {
            fwBadge.className = 'fw-badge current';
            fwBadge.title = 'Firmware up to date (' + msg.fw_version + ')';
        } else {
            fwBadge.className = 'fw-badge unknown';
            fwBadge.title = 'Firmware version unknown';
        }
    }
}

// --- Engage toggle UI ---
function updateEngageToggle(engaged, ramping, cutoff, absent) {
    var btn = document.getElementById("btn-engage");
    if (!btn) return;
    // cutoff only blocks engagement when battery is physically present
    var realCutoff = cutoff && !absent;
    btn.disabled = ramping || realCutoff;
    if (realCutoff) {
        btn.textContent = "BATTERY CUTOFF \u2014 REBOOT";
        btn.className = "engage-toggle cutoff";
    } else if (ramping) {
        btn.textContent = absent ? "RAMPING\u2026 (USB)" : "RAMPING\u2026";
        btn.className = "engage-toggle ramping";
    } else if (engaged) {
        btn.textContent = absent ? "SERVOS: ENGAGED (USB)" : "SERVOS: ENGAGED";
        btn.className = "engage-toggle engaged";
    } else {
        btn.textContent = absent ? "SERVOS: DISENGAGED (USB)" : "SERVOS: DISENGAGED";
        btn.className = "engage-toggle disengaged";
    }
    // Disable/enable action buttons
    var motionBtns = document.querySelectorAll(".dpad-btn, .action-btn[data-action]:not([data-action='balance-toggle'])");
    motionBtns.forEach(function(b) {
        b.disabled = !engaged || ramping || realCutoff;
    });
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
    } else if (msg.type === "telem_joints") {
        Dog3D.updateJoints(msg);
    } else if (msg.type === "telem_odometry") {
        Dog3D.updateOdometry(msg);
        updateMotionIndicator(msg.motion);
        if (msg.heading != null) {
            var h = Math.round(msg.heading) % 360;
            if (h < 0) h += 360;
            document.getElementById("heading-val").textContent = h + "\u00B0";
        }
    } else if (msg.type === "telem_status") {
        updateStatus(msg);
        if (msg.ota_status) updateOtaStatus(msg.ota_status, msg.ota_error);
    } else if (msg.type === "balance_state") {
        if (actionsCtrl) actionsCtrl.setBalanceState(msg.enabled);
    } else if (msg.type === "telem_sonar") {
        updateUltrasonic(msg.distance_mm);
    } else if (msg.type === "event_fall") {
        showFallAlert(true); Dog3D.setFallen(true);
    } else if (msg.type === "event_recovered") {
        showFallAlert(false); Dog3D.setFallen(false);
    } else if (msg.type === "lock_status") {
        updateLockUI(msg);
    } else if (msg.type === "lock_challenge") {
        if (confirm(msg.challenger + " wants control. Yield?")) {
            send({ type: "cmd_lock_yield" });
        }
    } else if (msg.type === "lock_denied") {
        var el = document.getElementById("fall-alert");
        el.textContent = msg.holder ? "Control held by " + msg.holder : "Control request denied";
        el.classList.remove("hidden");
        setTimeout(function() { el.classList.add("hidden"); }, 3000);
    } else if (msg.type === "telem_battery") {
        if (msg.present !== false) {
            recordBattery(msg.pct);
            document.getElementById("battery-val").textContent = msg.pct + "%";
        } else {
            document.getElementById("battery-val").textContent = "USB";
        }
    } else if (msg.type === 'telem_servo_pins') {
        updateServoPins(msg);
    } else if (msg.type === "ack" && msg.ok === false) {
        var el = document.getElementById("fall-alert");
        el.textContent = (msg.ref_type || "command") + " rejected";
        el.classList.remove("hidden");
        setTimeout(function() { el.classList.add("hidden"); }, 3000);
    } else if (msg.type === "telem_event") {
        var ev = msg.event;
        if (ev === "battery_cutoff_detach") {
            updateEngageToggle(false, false, true);
            setEngaged(false, false);
        } else if (ev === "battery_absent_clear_latch") {
            // USB-only: stale latch cleared; keep engage enabled
            updateEngageToggle(false, false, false, true);
            setEngaged(false, false);
        } else if (ev === "heartbeat_detach") {
            updateEngageToggle(false, false, false);
            setEngaged(false, false);
        } else if (ev === "engage_complete") {
            updateEngageToggle(true, false, false);
            setEngaged(true, false);
        } else if (ev === "disengage_complete") {
            updateEngageToggle(false, false, false);
            setEngaged(false, false);
        }
    } else if (msg.type === "reset") {
        Dog3D.reset();
    } else if (msg.type === "version") {
        if (window._appVersion && msg.hash !== window._appVersion) location.reload();
        window._appVersion = msg.hash;
    }
    diagHandleTelem(msg);
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

function setupEngage() {
    var btn = document.getElementById("btn-engage");
    if (!btn) return;
    btn.addEventListener("click", function () {
        var engaged = btn.classList.contains("engaged");
        send({ type: "cmd_engage", enabled: !engaged });
    });
}

// --- Init ---
Dog3D.init("dog-3d-container");
diagInit();
setMessageHandler(handleMessage);

actionsCtrl = setupActions(Dog3D);
setupDpad();
setEngaged(false, false);  // dim D-pad until engaged
updateEngageToggle(false, false, false);
setupKeyboard();
setupLock();
setupEngage();
setupReset();
setupBatteryGraph();
setupOtaPanel();
initOffsetPanel(send);
initGaitPanel(send);
initTransformPanel(send);
initServoNudgePanel(send);

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

connect(function () {
    send({ type: "cmd_identify", name: operatorName });
});
