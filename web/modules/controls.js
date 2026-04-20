// D-pad, keyboard, and action controls
import { send } from './ws.js';
import { TRANSFORM_POSES } from './poses.js';
import { setTransformSliders } from './panels.js';

var canControlFn = null;
export function setCanControl(fn) { canControlFn = fn; }
function canControl() { return canControlFn ? canControlFn() : true; }

var _engaged = false;
var _ramping = false;
export function setEngaged(engaged, ramping) {
    _engaged = engaged;
    _ramping = ramping;
    var dpad = document.querySelector('.dpad');
    if (dpad) dpad.classList.toggle('dpad-disabled', !engaged || ramping);
}

var balanceEnabled = false;

export function setupDpad() {
    document.querySelectorAll(".dpad-btn").forEach(function (btn) {
        var dir = btn.dataset.dir;

        btn.addEventListener("touchstart", function (e) {
            e.preventDefault();
            if (!canControl() || !_engaged || _ramping) return;
            btn.classList.add("pressed");
            send({ type: "cmd_move", direction: dir });
        });
        btn.addEventListener("touchend", function (e) {
            e.preventDefault();
            btn.classList.remove("pressed");
            if (dir !== "stop") send({ type: "cmd_move", direction: "stop" });
        });
        btn.addEventListener("mousedown", function () {
            if (!canControl() || !_engaged || _ramping) return;
            btn.classList.add("pressed");
            send({ type: "cmd_move", direction: dir });
        });
        btn.addEventListener("mouseup", function () {
            btn.classList.remove("pressed");
            if (dir !== "stop") send({ type: "cmd_move", direction: "stop" });
        });
        btn.addEventListener("mouseleave", function () {
            if (btn.classList.contains("pressed")) {
                btn.classList.remove("pressed");
                send({ type: "cmd_move", direction: "stop" });
            }
        });
    });
}

export function setupKeyboard() {
    var keyMap = {
        ArrowUp: "forward", ArrowDown: "backward", ArrowLeft: "left", ArrowRight: "right",
        w: "forward", s: "backward", a: "left", d: "right",
    };
    var pressed = {};

    document.addEventListener("keydown", function (e) {
        var el = document.activeElement;
        var tag = el ? el.tagName : "";
        if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
        if (!canControl() || !_engaged || _ramping) return;
        var dir = keyMap[e.key];
        if (dir && !pressed[e.key]) {
            pressed[e.key] = true;
            send({ type: "cmd_move", direction: dir });
            var btn = document.querySelector('[data-dir="' + dir + '"]');
            if (btn) btn.classList.add("pressed");
        }
        if (e.key === "b") { send({ type: "cmd_balance", enabled: !balanceEnabled }); }
        if (e.key === "e" || e.key === "E") {
            var btn = document.getElementById("btn-engage");
            if (btn && !btn.disabled) btn.click();
        }
    });

    document.addEventListener("keyup", function (e) {
        var el = document.activeElement;
        var tag = el ? el.tagName : "";
        if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
        var dir = keyMap[e.key];
        if (dir && pressed[e.key]) {
            delete pressed[e.key];
            var btn = document.querySelector('[data-dir="' + dir + '"]');
            if (btn) btn.classList.remove("pressed");
            var remaining = Object.keys(pressed).filter(function (k) { return keyMap[k]; });
            if (remaining.length > 0) {
                send({ type: "cmd_move", direction: keyMap[remaining[remaining.length - 1]] });
            } else {
                send({ type: "cmd_move", direction: "stop" });
            }
        }
    });
}

export function setupActions(Dog3D) {
    document.querySelectorAll(".action-btn[data-action]").forEach(function (btn) {
        btn.addEventListener("click", function () {
            if (!canControl()) return;
            var action = btn.dataset.action;
            if (action === "balance-toggle") {
                send({ type: "cmd_balance", enabled: !balanceEnabled });
            } else if (action.indexOf("pose-") === 0) {
                var poseName = action.slice(5);
                var payload = TRANSFORM_POSES[poseName];
                if (payload === undefined) return;
                send(Object.assign({ type: "cmd_transform", ms: 400 }, payload));
                setTransformSliders(payload);
                if (Dog3D && Dog3D.setPose) Dog3D.setPose(poseName);
            }
        });
    });

    return {
        setBalanceState: function (enabled) {
            balanceEnabled = enabled;
            var balVal = document.getElementById("balance-val");
            var balBtn = document.getElementById("btn-balance");
            if (enabled) {
                balVal.textContent = "ON"; balVal.className = "status-value on";
                balBtn.textContent = "Balance: ON"; balBtn.classList.add("active");
            } else {
                balVal.textContent = "OFF"; balVal.className = "status-value off";
                balBtn.textContent = "Balance: OFF"; balBtn.classList.remove("active");
            }
        }
    };
}
