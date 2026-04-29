// WebSocket connection management
//
// Reconnect strategy: exponential backoff (1, 2, 4, 8, max 16 s) with full handler
// detach on close. Each `connect()` builds a fresh WebSocket and clears the prior
// one's listeners — failed attempts cannot leave stale closures pinning the old
// socket or racing with `onopen` of the new one.
//
// `setConnected(false)` dispatches a `connection-lost` event on `window` so any
// module that paints live values can mark them stale; `connection-restored`
// fires on reconnect.

import { dispatch } from './bus.js';

const WS_URL = "ws://" + location.host + "/ws";
const RECONNECT_MIN_MS = 1000;
const RECONNECT_MAX_MS = 16000;

let ws = null;
let savedOnOpen = null;
let reconnectTimer = null;
let reconnectDelay = RECONNECT_MIN_MS;
let lastConnected = false;

export function connect(onOpen) {
    if (onOpen) savedOnOpen = onOpen;
    if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) {
        return;
    }
    detachSocket(ws);
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }

    const sock = new WebSocket(WS_URL);
    ws = sock;

    sock.onopen = function () {
        if (sock !== ws) return;  // superseded by a newer attempt
        reconnectDelay = RECONNECT_MIN_MS;
        setConnected(true);
        if (savedOnOpen) savedOnOpen();
    };
    sock.onclose = function () {
        if (sock !== ws) { detachSocket(sock); return; }
        detachSocket(sock);
        ws = null;
        setConnected(false);
        scheduleReconnect();
    };
    sock.onerror = function () {
        // onerror is always followed by onclose; let onclose drive reconnect.
        if (sock === ws) sock.close();
    };
    sock.onmessage = function (event) {
        if (sock !== ws) return;
        try {
            dispatch(JSON.parse(event.data));
        } catch (e) {
            console.warn("[ws] message parse error:", e, event.data);
        }
    };
}

function detachSocket(sock) {
    if (!sock) return;
    sock.onopen = null;
    sock.onclose = null;
    sock.onerror = null;
    sock.onmessage = null;
}

function scheduleReconnect() {
    if (reconnectTimer) return;
    const delay = reconnectDelay;
    reconnectDelay = Math.min(reconnectDelay * 2, RECONNECT_MAX_MS);
    reconnectTimer = setTimeout(function () {
        reconnectTimer = null;
        connect();
    }, delay);
}

export function send(msg) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(msg));
    }
}

function setConnected(connected) {
    const indicator = document.getElementById("conn-indicator");
    const text = document.getElementById("conn-text");
    if (indicator) indicator.className = "indicator " + (connected ? "connected" : "disconnected");
    if (text) text.textContent = connected ? "Online" : "Offline";

    if (connected !== lastConnected) {
        lastConnected = connected;
        window.dispatchEvent(new CustomEvent(
            connected ? "connection-restored" : "connection-lost"));
    }
}
