// Bark-Buddy Web UI — ES module entry point
import Dog3D from './dog3d/index.js';
import { connect, send } from './modules/ws.js';
import { on } from './modules/bus.js';
import { setupDpad, setupKeyboard, setupActions, setCanControl, setEngaged } from './modules/controls.js';
import { setupBatteryGraph, recordBattery,
         setupOtaPanel, updateOtaStatus,
         initOffsetPanel, updateServoPins,
         initGaitPanel, initTransformPanel,
         initServoNudgePanel } from './modules/panels.js';
import { diagInit } from './modules/diag.js';

// --- State ---
var hasLock = false;
var lockHolder = null;
var operatorName = new URLSearchParams(location.search).get("name") || "Operator";
var lastStatus = {};  // most recent telem_status — read by event branch for accurate engage UI

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

// --- Banner helper ---
// Single shared banner element with a single timer — guards against rapid
// successive transient messages stomping each other and leaving the banner
// hung visible after the last message's timer fires.
var bannerTimer = null;
function flashBanner(text, ms) {
    var el = document.getElementById("fall-alert");
    if (!el) return;
    if (bannerTimer) { clearTimeout(bannerTimer); bannerTimer = null; }
    el.textContent = text;
    el.classList.remove("hidden");
    if (ms > 0) {
        bannerTimer = setTimeout(function () {
            el.classList.add("hidden");
            bannerTimer = null;
        }, ms);
    }
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
    // Cache for event handlers that need to read absent/cutoff state.
    Object.assign(lastStatus, msg);
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

// --- Bus subscriptions ---
// Each module subscribes to the message types it cares about. Multiple
// subscribers per type are allowed; new features (SLAM, mapping, ...)
// register their own handlers without touching this file.

on('telem_imu', function (msg) {
    updateGauge('pitch', msg.pitch);
    updateGauge('roll', msg.roll);
});

on('telem_odometry', function (msg) {
    updateMotionIndicator(msg.motion);
    if (msg.heading != null) {
        var h = Math.round(msg.heading) % 360;
        if (h < 0) h += 360;
        document.getElementById('heading-val').textContent = h + '\u00B0';
    }
});

on('telem_status', function (msg) {
    updateStatus(msg);
    if (msg.ota_status) updateOtaStatus(msg.ota_status, msg.ota_error);
});

on('balance_state', function (msg) {
    if (actionsCtrl) actionsCtrl.setBalanceState(msg.enabled);
});

on('telem_sonar',     function (msg) { updateUltrasonic(msg.distance_mm); });
on('event_fall',      function ()    { showFallAlert(true); });
on('event_recovered', function ()    { showFallAlert(false); });

on('lock_status', updateLockUI);
on('lock_challenge', function (msg) {
    if (confirm(msg.challenger + ' wants control. Yield?')) {
        send({ type: 'cmd_lock_yield' });
    }
});
on('lock_denied', function (msg) {
    flashBanner(msg.holder ? 'Control held by ' + msg.holder : 'Control request denied', 3000);
});

on('telem_battery', function (msg) {
    if (msg.present !== false) {
        recordBattery(msg.pct);
        document.getElementById('battery-val').textContent = msg.pct + '%';
    } else {
        document.getElementById('battery-val').textContent = 'USB';
    }
});

on('telem_servo_pins', updateServoPins);

on('ack', function (msg) {
    if (msg.ok === false) flashBanner((msg.ref_type || 'command') + ' rejected', 3000);
});

on('telem_event', function (msg) {
    // Event-driven engage updates: read absent from cached telem_status so a
    // USB-only board with a transient ADC dip doesn't lock the engage button
    // to BATTERY CUTOFF between event and the next 1 Hz status.
    var absent = lastStatus.battery_present === false;
    switch (msg.event) {
        case 'battery_cutoff_detach':
            updateEngageToggle(false, false, true, absent);  setEngaged(false, false); break;
        case 'battery_absent_clear_latch':
            updateEngageToggle(false, false, false, true);    setEngaged(false, false); break;
        case 'heartbeat_detach':
            updateEngageToggle(false, false, false, absent); setEngaged(false, false); break;
        case 'engage_complete':
            updateEngageToggle(true,  false, false, absent); setEngaged(true,  false); break;
        case 'disengage_complete':
            updateEngageToggle(false, false, false, absent); setEngaged(false, false); break;
        case 'tilt_fault':
            flashBanner('TILT FAULT \u2014 gait halted', 4000); break;
    }
});

on('version', function (msg) {
    if (window._appVersion && msg.hash !== window._appVersion) location.reload();
    window._appVersion = msg.hash;
});

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

// Stale-telemetry indication: when WS drops, dim live readouts so the
// operator does not mistake a frozen value for current state.
var STALE_NODE_IDS = [
    "pitch-val", "roll-val", "battery-val", "ultra-val",
    "heading-val", "motion-state", "fw-badge", "transport-badge",
];
function setStale(stale) {
    STALE_NODE_IDS.forEach(function (id) {
        var el = document.getElementById(id);
        if (el) el.classList.toggle("stale", stale);
    });
    var container = document.getElementById("dog-3d-container");
    if (container) container.classList.toggle("stale", stale);
}
window.addEventListener("connection-lost", function () { setStale(true); });
window.addEventListener("connection-restored", function () { setStale(false); });

connect(function () {
    send({ type: "cmd_identify", name: operatorName });
});
