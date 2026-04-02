// Bark-Buddy Web UI
// Connects to Python host via WebSocket, sends D-pad commands, displays telemetry.

(function () {
    "use strict";

    // --- State ---
    var balanceEnabled = false;

    // --- WebSocket connection ---
    var ws = null;
    var reconnectTimer = null;
    var WS_URL = "ws://" + location.host + "/ws";

    function connect() {
        if (ws && ws.readyState <= WebSocket.OPEN) return;

        ws = new WebSocket(WS_URL);

        ws.onopen = function () {
            setConnected(true);
            if (reconnectTimer) {
                clearInterval(reconnectTimer);
                reconnectTimer = null;
            }
        };

        ws.onclose = function () {
            setConnected(false);
            scheduleReconnect();
        };

        ws.onerror = function () {
            ws.close();
        };

        ws.onmessage = function (event) {
            try {
                handleMessage(JSON.parse(event.data));
            } catch (e) {
                // Ignore malformed messages
            }
        };
    }

    function scheduleReconnect() {
        if (reconnectTimer) return;
        reconnectTimer = setInterval(connect, 2000);
    }

    function send(msg) {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify(msg));
        }
    }

    // --- Connection status ---
    function setConnected(connected) {
        var indicator = document.getElementById("conn-indicator");
        var text = document.getElementById("conn-text");
        if (connected) {
            indicator.className = "indicator connected";
            text.textContent = "Connected";
        } else {
            indicator.className = "indicator disconnected";
            text.textContent = "Disconnected";
        }
    }

    // --- Incoming messages ---
    function handleMessage(msg) {
        if (msg.type === "telem_imu") {
            updateGauge("pitch", msg.pitch);
            updateGauge("roll", msg.roll);
        } else if (msg.type === "telem_status") {
            updateStatus(msg);
        } else if (msg.type === "balance_state") {
            setBalanceState(msg.enabled);
        } else if (msg.type === "telem_ultrasonic") {
            updateUltrasonic(msg.distance_mm);
        } else if (msg.type === "event_fall") {
            showFallAlert(true);
        } else if (msg.type === "event_recovered") {
            showFallAlert(false);
        } else if (msg.type === "patrol_position") {
            updatePatrolPosition(msg);
        } else if (msg.type === "patrol_waypoint") {
            document.getElementById("patrol-wp").textContent =
                "Reached WP " + msg.index + " (" + msg.x.toFixed(1) + ", " + msg.y.toFixed(1) + ")";
        } else if (msg.type === "patrol_complete") {
            setPatrolRunning(false);
        }
    }

    function updateGauge(name, value) {
        var fill = document.getElementById(name + "-fill");
        var val = document.getElementById(name + "-val");
        val.textContent = value.toFixed(1) + "\u00B0";

        // Map value to gauge position: -45..+45 degrees, center = 50%
        var clamped = Math.max(-45, Math.min(45, value));
        var pct = (clamped / 45) * 50;
        if (pct >= 0) {
            fill.style.left = "50%";
            fill.style.width = pct + "%";
        } else {
            fill.style.left = (50 + pct) + "%";
            fill.style.width = (-pct) + "%";
        }

        // Color thresholds
        var absPct = Math.abs(pct);
        if (absPct > 38) {
            fill.className = "gauge-fill danger";
        } else if (absPct > 22) {
            fill.className = "gauge-fill warn";
        } else {
            fill.className = "gauge-fill";
        }
    }

    function updateStatus(msg) {
        if (msg.battery_mv != null) {
            var pct = batteryPercent(msg.battery_mv);
            document.getElementById("battery-val").textContent = pct + "%";
        }
        if (msg.mode != null) {
            document.getElementById("mode-val").textContent = msg.mode;
            setPatrolRunning(msg.mode === "patrol");
        }
        if (msg.balance != null) {
            setBalanceState(msg.balance);
        }
        if (msg.fallen != null) {
            showFallAlert(msg.fallen);
        }
    }

    function setBalanceState(enabled) {
        balanceEnabled = enabled;
        var balVal = document.getElementById("balance-val");
        var balBtn = document.getElementById("btn-balance");
        if (enabled) {
            balVal.textContent = "ON";
            balVal.className = "status-value on";
            balBtn.textContent = "Balance: ON";
            balBtn.classList.add("active");
        } else {
            balVal.textContent = "OFF";
            balVal.className = "status-value off";
            balBtn.textContent = "Balance: OFF";
            balBtn.classList.remove("active");
        }
    }

    function showFallAlert(fallen) {
        var el = document.getElementById("fall-alert");
        if (fallen) {
            el.classList.remove("hidden");
        } else {
            el.classList.add("hidden");
        }
    }

    function updateUltrasonic(mm) {
        var el = document.getElementById("ultra-val");
        if (mm < 100) {
            el.textContent = mm + "mm";
            el.className = "status-value danger-text";
        } else if (mm < 300) {
            el.textContent = mm + "mm";
            el.className = "status-value warn-text";
        } else {
            el.textContent = mm + "mm";
            el.className = "status-value";
        }
    }

    function batteryPercent(mv) {
        // 2S LiPo: 6000mV = 0%, 8400mV = 100%
        var pct = Math.round(((mv - 6000) / 2400) * 100);
        return Math.max(0, Math.min(100, pct));
    }

    // --- D-pad controls ---
    function setupDpad() {
        var buttons = document.querySelectorAll(".dpad-btn");
        buttons.forEach(function (btn) {
            var dir = btn.dataset.dir;

            // Touch events (mobile)
            btn.addEventListener("touchstart", function (e) {
                e.preventDefault();
                btn.classList.add("pressed");
                send({ type: "cmd_move", direction: dir });
            });

            btn.addEventListener("touchend", function (e) {
                e.preventDefault();
                btn.classList.remove("pressed");
                if (dir !== "stop") {
                    send({ type: "cmd_move", direction: "stop" });
                }
            });

            // Mouse events (desktop)
            btn.addEventListener("mousedown", function () {
                btn.classList.add("pressed");
                send({ type: "cmd_move", direction: dir });
            });

            btn.addEventListener("mouseup", function () {
                btn.classList.remove("pressed");
                if (dir !== "stop") {
                    send({ type: "cmd_move", direction: "stop" });
                }
            });

            btn.addEventListener("mouseleave", function () {
                if (btn.classList.contains("pressed")) {
                    btn.classList.remove("pressed");
                    send({ type: "cmd_move", direction: "stop" });
                }
            });
        });
    }

    // --- Action buttons ---
    function setupActions() {
        document.querySelectorAll(".action-btn").forEach(function (btn) {
            btn.addEventListener("click", function () {
                var action = btn.dataset.action;
                if (action === "stand") {
                    send({ type: "cmd_stand" });
                } else if (action === "balance-toggle") {
                    send({ type: "cmd_balance", enabled: !balanceEnabled });
                } else if (action === "wave") {
                    send({ type: "cmd_action", action: 1 });
                } else if (action === "sit") {
                    send({ type: "cmd_action", action: 4 });
                } else if (action === "lie") {
                    send({ type: "cmd_action", action: 5 });
                }
            });
        });
    }

    // --- Keyboard controls ---
    function setupKeyboard() {
        var keyMap = {
            ArrowUp: "forward",
            ArrowDown: "backward",
            ArrowLeft: "left",
            ArrowRight: "right",
            w: "forward",
            s: "backward",
            a: "left",
            d: "right",
        };

        var pressed = {};

        document.addEventListener("keydown", function (e) {
            var dir = keyMap[e.key];
            if (dir && !pressed[e.key]) {
                pressed[e.key] = true;
                send({ type: "cmd_move", direction: dir });
                var btn = document.querySelector('[data-dir="' + dir + '"]');
                if (btn) btn.classList.add("pressed");
            }
            if (e.key === " ") {
                e.preventDefault();
                send({ type: "cmd_move", direction: "stop" });
            }
            if (e.key === "b") {
                send({ type: "cmd_balance", enabled: !balanceEnabled });
            }
        });

        document.addEventListener("keyup", function (e) {
            var dir = keyMap[e.key];
            if (dir && pressed[e.key]) {
                delete pressed[e.key];
                var btn = document.querySelector('[data-dir="' + dir + '"]');
                if (btn) btn.classList.remove("pressed");

                // Find if another direction key is still held
                var remaining = Object.keys(pressed).filter(function (k) { return keyMap[k]; });
                if (remaining.length > 0) {
                    // Re-send the most recently pressed direction
                    send({ type: "cmd_move", direction: keyMap[remaining[remaining.length - 1]] });
                } else {
                    send({ type: "cmd_move", direction: "stop" });
                }
            }
        });
    }

    // --- Patrol controls ---
    function setupPatrol() {
        var btnDemo = document.getElementById("btn-patrol-demo");
        var btnStop = document.getElementById("btn-patrol-stop");

        // Demo patrol: a small square (0.5m sides)
        btnDemo.addEventListener("click", function () {
            send({
                type: "cmd_patrol",
                action: "start",
                waypoints: [
                    { x: 0.5, y: 0, heading: 0 },
                    { x: 0.5, y: 0.5, heading: 90 },
                    { x: 0, y: 0.5, heading: 180 },
                    { x: 0, y: 0, heading: 270 },
                ],
            });
        });

        btnStop.addEventListener("click", function () {
            send({ type: "cmd_patrol", action: "stop" });
        });
    }

    function setPatrolRunning(running) {
        var btnDemo = document.getElementById("btn-patrol-demo");
        var btnStop = document.getElementById("btn-patrol-stop");
        var status = document.getElementById("patrol-status");

        btnDemo.disabled = running;
        btnStop.disabled = !running;
        if (running) {
            status.classList.remove("hidden");
        } else {
            status.classList.add("hidden");
        }
    }

    function updatePatrolPosition(pos) {
        document.getElementById("patrol-pos").textContent =
            "Position: (" + pos.x.toFixed(2) + ", " + pos.y.toFixed(2) + ") " +
            pos.heading.toFixed(0) + "\u00B0";
    }

    // --- Init ---
    setupDpad();
    setupActions();
    setupKeyboard();
    setupPatrol();
    connect();
})();
