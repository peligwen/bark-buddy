// web/modules/panels/battery.js — battery history graph panel.

const battHistory = [];
const BATT_MAX_POINTS = 300;

export function setupBatteryGraph() {
    document.getElementById('batt-stat').addEventListener('click', function (e) {
        e.stopPropagation();
        const popup = document.getElementById('batt-popup');
        popup.classList.toggle('hidden');
        if (!popup.classList.contains('hidden')) drawBattGraph();
    });
    document.getElementById('batt-popup-close').addEventListener('click', function () {
        document.getElementById('batt-popup').classList.add('hidden');
    });
}

export function recordBattery(pct) {
    const now = Date.now() / 1000;
    battHistory.push({ time: now, pct });
    if (battHistory.length > BATT_MAX_POINTS) battHistory.shift();
    if (!document.getElementById('batt-popup').classList.contains('hidden')) drawBattGraph();
}

function drawBattGraph() {
    const canvas = document.getElementById('batt-canvas');
    const rect = canvas.parentElement.getBoundingClientRect();
    canvas.width = Math.floor(rect.width); canvas.height = 120;
    const ctx = canvas.getContext('2d');
    const w = canvas.width - 16, h = canvas.height - 16, ox = 8, oy = 8;

    ctx.fillStyle = '#151820'; ctx.fillRect(0, 0, canvas.width, canvas.height);

    if (battHistory.length < 2) {
        ctx.fillStyle = '#4b5160'; ctx.font = '11px monospace';
        ctx.fillText('Waiting for data...', ox + 4, oy + h / 2); return;
    }

    const tMin = battHistory[0].time, tMax = battHistory[battHistory.length - 1].time;
    const tRange = Math.max(tMax - tMin, 1);

    ctx.strokeStyle = '#252a35'; ctx.lineWidth = 1;
    for (let g = 25; g <= 75; g += 25) {
        const gy = oy + h - (g / 100) * h;
        ctx.beginPath(); ctx.moveTo(ox, gy); ctx.lineTo(ox + w, gy); ctx.stroke();
    }

    ctx.beginPath(); ctx.lineWidth = 2;
    for (let i = 0; i < battHistory.length; i++) {
        const pt = battHistory[i];
        const x = ox + ((pt.time - tMin) / tRange) * w;
        const y = oy + h - (Math.max(0, Math.min(100, pt.pct)) / 100) * h;
        ctx.strokeStyle = pt.pct < 20 ? '#ef4444' : pt.pct < 40 ? '#f59e0b' : '#22c55e';
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();

    const last = battHistory[battHistory.length - 1];
    ctx.fillStyle = '#e0e0e0'; ctx.font = '11px monospace';
    ctx.fillText(last.pct + '%', ox + w - 30, oy + 12);
    const mins = Math.floor(tRange / 60), secs = Math.floor(tRange % 60);
    ctx.fillStyle = '#4b5160'; ctx.font = '9px monospace';
    ctx.fillText(mins > 0 ? mins + 'm ' + secs + 's' : secs + 's', ox + 2, oy + h - 2);
}
