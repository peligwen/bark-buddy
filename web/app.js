// Bark-Buddy Web UI
// Connects to Python host via WebSocket, sends D-pad commands, displays telemetry.

(function () {
    "use strict";

    // --- WebSocket connection ---
    let ws = null;
    let reconnectTimer = null;
    const WS_URL = `ws://${location.host}/ws`;

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
            handleMessage(JSON.parse(event.data));
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
            if (msg.battery_mv != null) {
                var pct = batteryPercent(msg.battery_mv);
                document.getElementById("battery-text").textContent = pct + "%";
            }
        }
    }

    function updateGauge(name, value) {
        var fill = document.getElementById(name + "-fill");
        var val = document.getElementById(name + "-val");
        val.textContent = value.toFixed(1);

        // Map value to gauge position: -45..+45 degrees, center = 50%
        var clamped = Math.max(-45, Math.min(45, value));
        var pct = (clamped / 45) * 50; // -50% to +50% from center
        if (pct >= 0) {
            fill.style.left = "50%";
            fill.style.width = pct + "%";
        } else {
            fill.style.left = (50 + pct) + "%";
            fill.style.width = (-pct) + "%";
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
                } else if (action === "balance-on") {
                    send({ type: "cmd_balance", enabled: true });
                } else if (action === "balance-off") {
                    send({ type: "cmd_balance", enabled: false });
                } else if (action === "wave") {
                    send({ type: "cmd_action", action: 1 });
                } else if (action === "sit") {
                    send({ type: "cmd_action", action: 4 });
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
                // Highlight corresponding button
                var btn = document.querySelector('[data-dir="' + dir + '"]');
                if (btn) btn.classList.add("pressed");
            }
            if (e.key === " ") {
                e.preventDefault();
                send({ type: "cmd_move", direction: "stop" });
            }
        });

        document.addEventListener("keyup", function (e) {
            var dir = keyMap[e.key];
            if (dir && pressed[e.key]) {
                delete pressed[e.key];
                send({ type: "cmd_move", direction: "stop" });
                var btn = document.querySelector('[data-dir="' + dir + '"]');
                if (btn) btn.classList.remove("pressed");
            }
        });
    }

    // --- Init ---
    setupDpad();
    setupActions();
    setupKeyboard();
    connect();
})();
