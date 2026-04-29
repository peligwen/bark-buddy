// web/modules/panels/servo_nudge.js — manual ±step on a single servo's pulse.

import { clamp } from './_util.js';

export function initServoNudgePanel(sendFn) {
    const pulses = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500];

    const minusBtn = document.getElementById('servo-nudge-minus');
    const plusBtn  = document.getElementById('servo-nudge-plus');
    if (!minusBtn || !plusBtn) return;

    function nudge(sign) {
        const idx  = parseInt(document.getElementById('servo-nudge-index').value, 10);
        const step = parseInt(document.getElementById('servo-nudge-step').value, 10);
        const next = clamp(pulses[idx] + sign * step, 500, 2500);
        pulses[idx] = next;
        const readout = document.getElementById('servo-nudge-readout-' + idx);
        if (readout) readout.textContent = next;
        sendFn({ type: 'cmd_servo', index: idx, pulse_us: next });
    }

    minusBtn.addEventListener('click', () => nudge(-1));
    plusBtn .addEventListener('click', () => nudge(+1));
}
