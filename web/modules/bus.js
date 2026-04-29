// web/modules/bus.js
//
// Tiny in-process pub/sub for inbound WebSocket messages.
//
// Multiple subscribers per type (so app.module.js can update readouts at
// the same time as dog3d updates the model). Wildcard `'*'` subscribers
// fire on every message — useful for diag panels.
//
// `on(type, fn)` returns an unsubscribe function. Errors in handlers are
// caught + logged so one broken consumer can't take the rest down.

const handlers = new Map();   // type -> Set<fn>

export function on(type, fn) {
    if (!handlers.has(type)) handlers.set(type, new Set());
    handlers.get(type).add(fn);
    return function unsubscribe() {
        const s = handlers.get(type);
        if (s) s.delete(fn);
    };
}

export function dispatch(msg) {
    const type = msg && msg.type;
    if (type) emit(type, msg);
    emit('*', msg);
}

function emit(type, msg) {
    const set = handlers.get(type);
    if (!set) return;
    for (const fn of set) {
        try { fn(msg); }
        catch (e) { console.warn('[bus] handler error for', type, ':', e); }
    }
}
