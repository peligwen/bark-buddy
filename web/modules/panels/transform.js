// web/modules/panels/transform.js — body-frame transform sliders (cmd_transform).

import { clamp, debounce } from './_util.js';

const XFORM_AXES = [
    { id: 'roll',  min: -15, max:  15 },
    { id: 'pitch', min: -15, max:  15 },
    { id: 'yaw',   min: -20, max:  20 },
    { id: 'x',     min: -25, max:  25 },
    { id: 'y',     min: -25, max:  25 },
    { id: 'z',     min: -25, max:  25 },
];

export function setTransformSliders(payload) {
    XFORM_AXES.forEach((ax) => {
        const val = (payload[ax.id] !== undefined) ? payload[ax.id] : 0;
        const slider = document.getElementById('xform-' + ax.id);
        const display = document.getElementById('xform-' + ax.id + '-val');
        if (slider) slider.value = val;
        if (display) display.textContent = parseFloat(val).toFixed(1);
    });
}

export function initTransformPanel(sendFn) {
    const sendTransform = debounce(function () {
        const msg = { type: 'cmd_transform' };
        XFORM_AXES.forEach((ax) => {
            const slider = document.getElementById('xform-' + ax.id);
            if (slider) msg[ax.id] = clamp(parseFloat(slider.value), ax.min, ax.max);
        });
        sendFn(msg);
    }, 100);

    XFORM_AXES.forEach((ax) => {
        const slider  = document.getElementById('xform-' + ax.id);
        const display = document.getElementById('xform-' + ax.id + '-val');
        if (!slider) return;
        slider.addEventListener('input', () => {
            if (display) display.textContent = parseFloat(slider.value).toFixed(1);
            sendTransform();
        });
    });

    const resetBtn = document.getElementById('xform-reset-btn');
    if (resetBtn) {
        resetBtn.addEventListener('click', () => {
            setTransformSliders({});
            sendFn({ type: 'cmd_transform' });
        });
    }
}
