// web/modules/panels/servo_offset.js — per-servo offset trim + GPIO reassignment.

export function initOffsetPanel(sendFn) {
    const offsetBtn = document.getElementById('offset-set-btn');
    const pinBtn    = document.getElementById('pin-set-btn');
    if (!offsetBtn || !pinBtn) return;

    offsetBtn.addEventListener('click', () => {
        const index     = parseInt(document.getElementById('offset-idx').value, 10);
        const offset_us = parseInt(document.getElementById('offset-val').value, 10);
        if (isNaN(index) || isNaN(offset_us)) return;
        sendFn({ type: 'cmd_offset', index, offset_us });
    });

    pinBtn.addEventListener('click', () => {
        const index = parseInt(document.getElementById('pin-idx').value, 10);
        const pin   = parseInt(document.getElementById('pin-val').value, 10);
        if (isNaN(index) || isNaN(pin)) return;
        sendFn({ type: 'cmd_servo_pin', index, pin });
    });
}

export function updateServoPins(msg) {
    const el = document.getElementById('servo-pins-display');
    if (!el) return;
    const pins = msg.pins || [];
    if (!pins.length) return;
    el.textContent = 'Pins: ' + pins.map((p, i) => `${i}->${p}`).join('  ');
}
