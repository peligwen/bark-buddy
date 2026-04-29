// web/modules/panels/ota.js — firmware OTA modal + progress.

export function setupOtaPanel() {
    const fwBadge   = document.getElementById('fw-badge');
    const modal     = document.getElementById('ota-modal');
    const startBtn  = document.getElementById('ota-start');
    const cancelBtn = document.getElementById('ota-cancel');

    fwBadge.addEventListener('click', async function () {
        if (!fwBadge.classList.contains('outdated')) return;
        try {
            const resp = await fetch('/api/firmware/status');
            const data = await resp.json();
            document.getElementById('ota-current').textContent   = data.current_version || '--';
            document.getElementById('ota-available').textContent = data.available_version || '--';
            startBtn.disabled = !data.can_ota;
            startBtn.title = data.can_ota ? '' : 'OTA requires WiFi connection';
        } catch (_) {
            document.getElementById('ota-current').textContent   = '--';
            document.getElementById('ota-available').textContent = '--';
        }
        document.getElementById('ota-error').classList.add('hidden');
        document.getElementById('ota-progress').classList.add('hidden');
        modal.classList.remove('hidden');
    });

    cancelBtn.addEventListener('click', function () { modal.classList.add('hidden'); });

    startBtn.addEventListener('click', async function () {
        startBtn.disabled = true;
        cancelBtn.disabled = true;
        document.getElementById('ota-progress').classList.remove('hidden');
        document.getElementById('ota-error').classList.add('hidden');
        setOtaProgress('building', 10);
        try {
            const resp = await fetch('/api/firmware/update', { method: 'POST' });
            const data = await resp.json();
            if (!data.ok) {
                showOtaError(data.error || 'Update failed');
                cancelBtn.disabled = false;
            }
            // Further progress comes via WebSocket ota_status messages
        } catch (e) {
            showOtaError('Network error: ' + e.message);
            cancelBtn.disabled = false;
        }
    });
}

export function updateOtaStatus(status, error) {
    if (status === 'downloading' || status === 'flashing') {
        const modal = document.getElementById('ota-modal');
        if (modal) modal.classList.remove('hidden');
    }
    const fill = document.getElementById('ota-progress-fill');
    const text = document.getElementById('ota-status-text');
    if (!fill || !text) return;
    switch (status) {
        case 'downloading':
            text.textContent = 'Downloading firmware...'; fill.style.width = '40%'; break;
        case 'flashing':
            text.textContent = 'Flashing...';             fill.style.width = '75%'; break;
        case 'complete':
            text.textContent = 'Complete! Rebooting...';  fill.style.width = '100%'; break;
        case 'failed':
            showOtaError(error || 'Flash failed');         break;
    }
}

function setOtaProgress(label, pct) {
    const fill = document.getElementById('ota-progress-fill');
    const text = document.getElementById('ota-status-text');
    if (fill) fill.style.width = pct + '%';
    if (text) text.textContent = label;
}

function showOtaError(msg) {
    const el = document.getElementById('ota-error');
    if (el) { el.textContent = msg; el.classList.remove('hidden'); }
    const start = document.getElementById('ota-start');
    if (start) start.disabled = false;
}
