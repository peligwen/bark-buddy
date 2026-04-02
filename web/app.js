// Bark-Buddy Web UI
// Connects to Python host via WebSocket, sends D-pad commands, displays telemetry.

(function () {
    "use strict";

    // --- State ---
    var balanceEnabled = false;
    var hasLock = false;
    var lockHolder = null;
    var operatorName = new URLSearchParams(location.search).get("name") || "Operator";

    function canControl() {
        // Block if client version doesn't match server
        if (window._appVersion && window._serverVersion && window._appVersion !== window._serverVersion) {
            location.reload();
            return false;
        }
        return lockHolder === null || hasLock;
    }

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
            // Identify this client and request existing data
            send({ type: "cmd_identify", name: operatorName });
            send({ type: "cmd_map", action: "get" });
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
            text.textContent = "Online";
        } else {
            indicator.className = "indicator disconnected";
            text.textContent = "Offline";
        }
    }

    // --- Incoming messages ---
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
            setBalanceState(msg.enabled);
        } else if (msg.type === "telem_ultrasonic") {
            updateUltrasonic(msg.distance_mm);
        } else if (msg.type === "event_fall") {
            showFallAlert(true);
            Dog3D.setFallen(true);
        } else if (msg.type === "event_recovered") {
            showFallAlert(false);
            Dog3D.setFallen(false);
        } else if (msg.type === "patrol_position") {
            updatePatrolPosition(msg);
        } else if (msg.type === "patrol_waypoint") {
            document.getElementById("patrol-wp").textContent =
                "Reached WP " + msg.index + " (" + msg.x.toFixed(1) + ", " + msg.y.toFixed(1) + ")";
        } else if (msg.type === "patrol_complete") {
            setPatrolRunning(false);
        } else if (msg.type === "scan_point") {
            addScanPoint(msg);
        } else if (msg.type === "scan_complete") {
            setScanRunning(false);
        } else if (msg.type === "map_data") {
            renderFullMap(msg);
        } else if (msg.type === "lock_status") {
            updateLockUI(msg);
        } else if (msg.type === "lock_denied") {
            // Could show a toast; for now just update UI
        } else if (msg.type === "lock_challenge") {
            if (confirm(msg.challenger + " wants control. Yield?")) {
                send({ type: "cmd_lock_yield" });
            }
        } else if (msg.type === "reset") {
            Dog3D.reset();
            // Reset local UI state
            setScanRunning(false);
            setPatrolRunning(false);
        } else if (msg.type === "transport_result") {
            updateTransportUI(msg);
        } else if (msg.type === "wifi_setup_result") {
            var badge = document.getElementById("transport-badge");
            if (msg.ok) {
                badge.className = "transport-badge live";
                badge.textContent = "wifi:" + msg.ip;
                showWifiBanner(msg.ip, "");
            } else {
                badge.className = "transport-badge sim";
                badge.textContent = document.getElementById("transport-badge").dataset.prev || "SIM";
            }
        } else if (msg.type === "version") {
            if (window._appVersion && msg.hash !== window._appVersion) {
                location.reload();
            }
            window._appVersion = msg.hash;
            window._serverVersion = msg.hash;
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
            setScanRunning(msg.mode === "scan");
        }
        if (msg.balance != null) {
            setBalanceState(msg.balance);
        }
        if (msg.fallen != null) {
            showFallAlert(msg.fallen);
        }
        if (msg.transport != null) {
            var badge = document.getElementById("transport-badge");
            var isSim = msg.transport === "sim";
            badge.textContent = isSim ? "SIM" : msg.transport;
            badge.className = "transport-badge " + (isSim ? "sim" : "live");
        }
        if (msg.wifi_available && msg.wifi_ip) {
            showWifiBanner(msg.wifi_ip, msg.wifi_ssid);
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
        Dog3D.updateUltrasonic(mm);
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
                if (!canControl()) return;
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
                if (!canControl()) return;
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
                if (!canControl()) return;
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
            if (!canControl()) return;
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

    // --- Motion indicator ---
    function updateMotionIndicator(motion) {
        var el = document.getElementById("motion-state");
        if (!motion || motion === "stop") {
            el.textContent = "";
            el.className = "motion-indicator";
        } else {
            el.textContent = motion.toUpperCase();
            el.className = "motion-indicator active";
        }
    }

    // --- Scan / Map ---
    var dogMapState = { x: 0, y: 0, heading: 0, motion: "stop" };
    var mapPoints = [];
    var mapScans = [];
    var mapBounds = { min_x: -2, max_x: 2, min_y: -2, max_y: 2 };

    function setupScan() {
        var btnStart = document.getElementById("btn-scan-start");
        var btnStop = document.getElementById("btn-scan-stop");
        var btnClear = document.getElementById("btn-map-clear");

        btnStart.addEventListener("click", function () {
            send({ type: "cmd_scan", action: "start" });
        });

        btnStop.addEventListener("click", function () {
            send({ type: "cmd_scan", action: "stop" });
        });

        btnClear.addEventListener("click", function () {
            send({ type: "cmd_map", action: "clear" });
        });

        drawMap();
    }

    function setScanRunning(running) {
        var btnStart = document.getElementById("btn-scan-start");
        var btnStop = document.getElementById("btn-scan-stop");
        var progress = document.getElementById("scan-progress");

        btnStart.disabled = running;
        btnStop.disabled = !running;
        if (running) {
            progress.classList.remove("hidden");
        } else {
            progress.classList.add("hidden");
        }
    }

    function addScanPoint(msg) {
        mapPoints.push({ x: msg.x, y: msg.y, distance_mm: msg.distance_mm });
        updateMapBoundsFromPoint(msg.x, msg.y);
        drawMap();

        // Update progress
        document.getElementById("scan-progress-fill").style.width = msg.progress + "%";
        document.getElementById("scan-progress-text").textContent = msg.progress + "%";
        document.getElementById("map-points").textContent = mapPoints.length + " pts";

        // Rebuild 3D walls from scan data
        Dog3D.setScanPoints(mapPoints);
    }

    function renderFullMap(data) {
        mapPoints = data.points || [];
        mapScans = data.scans || [];
        mapBounds = data.bounds || mapBounds;
        document.getElementById("map-points").textContent = (data.point_count || 0) + " pts";
        document.getElementById("map-scans").textContent = (data.scan_count || 0) + " scans";
        drawMap();
        Dog3D.setScanPoints(mapPoints);
    }

    function updateMapBoundsFromPoint(x, y) {
        var margin = 0.2;
        if (x - margin < mapBounds.min_x) mapBounds.min_x = x - margin;
        if (x + margin > mapBounds.max_x) mapBounds.max_x = x + margin;
        if (y - margin < mapBounds.min_y) mapBounds.min_y = y - margin;
        if (y + margin > mapBounds.max_y) mapBounds.max_y = y + margin;
    }

    function drawMap() {
        var canvas = document.getElementById("map-canvas");
        // Auto-size canvas to container
        var rect = canvas.parentElement.getBoundingClientRect();
        var info = document.getElementById("map-info");
        var progress = document.getElementById("scan-progress");
        var usedH = info.offsetHeight + (progress.classList.contains("hidden") ? 0 : progress.offsetHeight);
        var cw = Math.floor(rect.width);
        var ch = Math.floor(rect.height - usedH);
        if (cw > 0 && ch > 0 && (canvas.width !== cw || canvas.height !== ch)) {
            canvas.width = cw;
            canvas.height = ch;
        }

        var ctx = canvas.getContext("2d");
        var w = canvas.width;
        var h = canvas.height;

        // Clear — dark background
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

        // Grid lines
        ctx.strokeStyle = "#252a35";
        ctx.lineWidth = 1;
        var gridStep = gridStepForRange(range);
        var gx = Math.floor(mapBounds.min_x / gridStep) * gridStep;
        while (gx <= mapBounds.max_x) {
            var cx = toCanvasX(gx);
            ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke();
            gx += gridStep;
        }
        var gy = Math.floor(mapBounds.min_y / gridStep) * gridStep;
        while (gy <= mapBounds.max_y) {
            var cy = toCanvasY(gy);
            ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke();
            gy += gridStep;
        }

        // Origin cross
        ctx.strokeStyle = "#3a3f4a";
        ctx.lineWidth = 1;
        var ox = toCanvasX(0), oy = toCanvasY(0);
        ctx.beginPath(); ctx.moveTo(ox - 8, oy); ctx.lineTo(ox + 8, oy); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(ox, oy - 8); ctx.lineTo(ox, oy + 8); ctx.stroke();

        // Scan origins
        ctx.fillStyle = "rgba(212, 136, 14, 0.5)";
        for (var s = 0; s < mapScans.length; s++) {
            var sc = mapScans[s];
            var sx = toCanvasX(sc.x);
            var sy = toCanvasY(sc.y);
            ctx.beginPath();
            ctx.arc(sx, sy, 3, 0, Math.PI * 2);
            ctx.fill();
        }

        // Obstacle points
        for (var i = 0; i < mapPoints.length; i++) {
            var p = mapPoints[i];
            var px = toCanvasX(p.x);
            var py = toCanvasY(p.y);
            var d = p.distance_mm;
            if (d < 150) {
                ctx.fillStyle = "#ef4444";
            } else if (d < 400) {
                ctx.fillStyle = "#f59e0b";
            } else {
                ctx.fillStyle = "#22c55e";
            }
            ctx.beginPath();
            ctx.arc(px, py, 3, 0, Math.PI * 2);
            ctx.fill();
        }

        // Dog position + heading triangle
        var dx = toCanvasX(dogMapState.x);
        var dy = toCanvasY(dogMapState.y);
        var headingRad = dogMapState.heading * (Math.PI / 180); // canvas rotate is CW-positive, matching sim heading
        var triSize = 8;
        ctx.save();
        ctx.translate(dx, dy);
        ctx.rotate(headingRad);
        ctx.beginPath();
        ctx.moveTo(triSize, 0);                          // nose (forward = +X = right on canvas)
        ctx.lineTo(-triSize * 0.6, -triSize * 0.5);      // rear left
        ctx.lineTo(-triSize * 0.6,  triSize * 0.5);      // rear right
        ctx.closePath();
        ctx.fillStyle = "#d4880e";
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.restore();

        // Scale label
        ctx.fillStyle = "#4b5160";
        ctx.font = "10px monospace";
        ctx.fillText(gridStep.toFixed(1) + "m", 6, h - 6);
    }

    function gridStepForRange(range) {
        if (range < 1) return 0.2;
        if (range < 3) return 0.5;
        if (range < 6) return 1.0;
        return 2.0;
    }

    // --- Lock UI ---
    function updateLockUI(msg) {
        hasLock = msg.is_you;
        lockHolder = msg.holder;
        var btn = document.getElementById("btn-lock");
        var text = document.getElementById("lock-text");
        var header = document.querySelector("header");
        if (msg.is_you) {
            btn.textContent = "Release";
            btn.classList.add("locked");
            text.textContent = "You have control";
            header.className = "control-self";
        } else if (msg.locked) {
            btn.textContent = "Request";
            btn.classList.remove("locked");
            text.textContent = msg.holder + " has control";
            header.className = "control-other";
        } else {
            btn.textContent = "Take Control";
            btn.classList.remove("locked");
            text.textContent = "";
            header.className = "";
        }
    }

    function setupLock() {
        document.getElementById("btn-lock").addEventListener("click", function () {
            if (hasLock) {
                send({ type: "cmd_unlock" });
            } else {
                send({ type: "cmd_lock", name: operatorName });
            }
        });
    }

    function setupReset() {
        document.getElementById("btn-reset").addEventListener("click", function () {
            send({ type: "cmd_reset" });
        });
    }

    // --- Transport switching ---
    function setupTransport() {
        var badge = document.getElementById("transport-badge");
        var menu = document.getElementById("transport-menu");

        badge.addEventListener("click", function (e) {
            e.stopPropagation();
            menu.classList.toggle("hidden");
        });

        // Close menu on outside click
        document.addEventListener("click", function () {
            menu.classList.add("hidden");
        });

        menu.addEventListener("click", function (e) {
            var btn = e.target.closest("[data-transport]");
            if (!btn) return;
            var mode = btn.dataset.transport;
            menu.classList.add("hidden");

            if (mode === "wifi") {
                var host = prompt("MechDog WiFi IP:", "192.168.1.163");
                if (!host) return;
                send({ type: "cmd_transport", mode: "wifi", wifi_host: host });
            } else if (mode === "wifi-setup") {
                var ssid = prompt("WiFi SSID:");
                if (!ssid) return;
                var pw = prompt("WiFi Password:");
                if (pw === null) return;
                send({ type: "cmd_wifi_setup", ssid: ssid, password: pw });
                badge.textContent = "SETUP...";
                badge.className = "transport-badge switching";
                return;
            } else {
                send({ type: "cmd_transport", mode: mode });
            }
            badge.textContent = "...";
            badge.className = "transport-badge switching";
        });
    }

    function updateTransportUI(msg) {
        var badge = document.getElementById("transport-badge");
        if (msg.ok) {
            // Badge will update from the next telem_status
            document.getElementById("wifi-banner").classList.add("hidden");
        } else {
            badge.textContent = msg.error || "Failed";
            badge.className = "transport-badge sim";
        }
    }

    function showWifiBanner(ip, ssid) {
        var banner = document.getElementById("wifi-banner");
        var text = document.getElementById("wifi-banner-text");
        text.textContent = "WiFi detected: " + (ssid || ip) + " (" + ip + ")";
        banner.classList.remove("hidden");

        document.getElementById("wifi-banner-switch").onclick = function () {
            send({ type: "cmd_transport", mode: "wifi", wifi_host: ip });
            banner.classList.add("hidden");
            var badge = document.getElementById("transport-badge");
            badge.textContent = "...";
            badge.className = "transport-badge switching";
        };
        document.getElementById("wifi-banner-dismiss").onclick = function () {
            banner.classList.add("hidden");
        };
    }

    // --- Init ---
    Dog3D.init("dog-3d-container");
    setupDpad();
    setupActions();
    setupKeyboard();
    setupPatrol();
    setupScan();
    setupLock();
    setupReset();
    setupTransport();
    connect();
})();
