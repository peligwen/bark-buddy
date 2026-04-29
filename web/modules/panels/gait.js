// web/modules/panels/gait.js — gait stride parameter sliders.

import { clamp, debounce } from './_util.js';

const GAIT_DEFAULTS = { stride_length: 12, stride_height: 10, frequency: 1.5 };

export function initGaitPanel(sendFn) {
    const sendGait = debounce(function () {
        const sl = clamp(parseFloat(document.getElementById('gait-stride-length').value), 0, 30);
        const sh = clamp(parseFloat(document.getElementById('gait-stride-height').value), 0, 20);
        const fr = clamp(parseFloat(document.getElementById('gait-frequency').value), 0.5, 3.0);
        sendFn({ type: 'cmd_gait_params', stride_length: sl, stride_height: sh, frequency: fr });
    }, 200);

    ['gait-stride-length', 'gait-stride-height', 'gait-frequency'].forEach((id) => {
        const el = document.getElementById(id);
        if (!el) return;
        const valEl = document.getElementById(id + '-val');
        const decimals = id === 'gait-frequency' ? 1 : 0;
        el.addEventListener('input', () => {
            if (valEl) valEl.textContent = parseFloat(el.value).toFixed(decimals);
            sendGait();
        });
    });

    const resetBtn = document.getElementById('gait-reset-btn');
    if (resetBtn) {
        resetBtn.addEventListener('click', () => {
            document.getElementById('gait-stride-length').value = GAIT_DEFAULTS.stride_length;
            document.getElementById('gait-stride-height').value = GAIT_DEFAULTS.stride_height;
            document.getElementById('gait-frequency').value     = GAIT_DEFAULTS.frequency;
            document.getElementById('gait-stride-length-val').textContent = GAIT_DEFAULTS.stride_length;
            document.getElementById('gait-stride-height-val').textContent = GAIT_DEFAULTS.stride_height;
            document.getElementById('gait-frequency-val').textContent     = GAIT_DEFAULTS.frequency.toFixed(1);
            sendFn({ type: 'cmd_gait_params', ...GAIT_DEFAULTS });
        });
    }
}
