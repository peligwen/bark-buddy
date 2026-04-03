// WebSocket connection management
var ws = null;
var reconnectTimer = null;
var WS_URL = "ws://" + location.host + "/ws";
var messageHandler = null;

export function setMessageHandler(fn) { messageHandler = fn; }

export function connect() {
    if (ws && ws.readyState <= WebSocket.OPEN) return;
    ws = new WebSocket(WS_URL);

    ws.onopen = function () {
        setConnected(true);
        if (reconnectTimer) { clearInterval(reconnectTimer); reconnectTimer = null; }
        if (window._onWsOpen) window._onWsOpen();
    };
    ws.onclose = function () { setConnected(false); scheduleReconnect(); };
    ws.onerror = function () { ws.close(); };
    ws.onmessage = function (event) {
        try {
            if (messageHandler) messageHandler(JSON.parse(event.data));
        } catch (e) {}
    };
}

function scheduleReconnect() {
    if (reconnectTimer) return;
    reconnectTimer = setInterval(connect, 2000);
}

export function send(msg) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(msg));
    }
}

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
