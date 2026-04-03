// Camera positioning and orbit controls.
import { state, BODY_L, BODY_H, standingHeight } from './state.js';

export function updateCameraPosition() {
    // Smoothly follow dog position
    state.camTargetX += (state.currentX - state.camTargetX) * 0.08;
    state.camTargetZ += (state.currentZ - state.camTargetZ) * 0.08;

    // Smoothly follow dog yaw (with angle wrapping)
    var yawDiff = state.currentYaw - state.camYaw;
    if (yawDiff > Math.PI) yawDiff -= 2 * Math.PI;
    if (yawDiff < -Math.PI) yawDiff += 2 * Math.PI;
    state.camYaw += yawDiff * 0.05;

    // Chase camera: orbit behind the dog
    // Dog faces +X at yaw=0. Camera at -X needs sin=-1,cos=0 -> angle=3pi/2
    var chaseAngle = state.camYaw + Math.PI * 1.5 + state.cameraAngle.thetaOffset;
    var r = state.cameraAngle.radius;
    var phi = state.cameraAngle.phi;

    // Look-at point: just forward and above the dog's nose
    var fwdOffset = BODY_L * 0.8; // forward from dog center
    var lookX = state.camTargetX + fwdOffset * Math.cos(state.camYaw);
    var lookZ = state.camTargetZ - fwdOffset * Math.sin(state.camYaw);
    var lookY = standingHeight() + BODY_H * 1.5;

    state.camera.position.x = lookX + r * Math.cos(phi) * Math.sin(chaseAngle);
    state.camera.position.y = lookY + r * Math.sin(phi);
    state.camera.position.z = lookZ + r * Math.cos(phi) * Math.cos(chaseAngle);
    state.camera.lookAt(lookX, lookY, lookZ);
}

export function setupControls() {
    var el = state.renderer.domElement;

    el.addEventListener("pointerdown", function (e) {
        state.isDragging = true;
        state.prevMouse.x = e.clientX;
        state.prevMouse.y = e.clientY;
        el.setPointerCapture(e.pointerId);
    });

    el.addEventListener("pointermove", function (e) {
        if (!state.isDragging) return;
        var dx = e.clientX - state.prevMouse.x;
        var dy = e.clientY - state.prevMouse.y;
        state.prevMouse.x = e.clientX;
        state.prevMouse.y = e.clientY;
        state.cameraAngle.thetaOffset -= dx * 0.008;
        state.cameraAngle.phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, state.cameraAngle.phi + dy * 0.008));
        updateCameraPosition();
    });

    el.addEventListener("pointerup", function () { state.isDragging = false; });

    el.addEventListener("wheel", function (e) {
        e.preventDefault();
        state.cameraAngle.radius = Math.max(1.5, Math.min(10, state.cameraAngle.radius + e.deltaY * 0.005));
        updateCameraPosition();
    }, { passive: false });

    var lastTouchDist = 0;
    el.addEventListener("touchstart", function (e) {
        if (e.touches.length === 2) {
            var dx = e.touches[0].clientX - e.touches[1].clientX;
            var dy = e.touches[0].clientY - e.touches[1].clientY;
            lastTouchDist = Math.sqrt(dx * dx + dy * dy);
        }
    }, { passive: true });
    el.addEventListener("touchmove", function (e) {
        if (e.touches.length === 2) {
            var dx = e.touches[0].clientX - e.touches[1].clientX;
            var dy = e.touches[0].clientY - e.touches[1].clientY;
            var dist = Math.sqrt(dx * dx + dy * dy);
            state.cameraAngle.radius = Math.max(1.5, Math.min(10, state.cameraAngle.radius + (lastTouchDist - dist) * 0.02));
            updateCameraPosition();
            lastTouchDist = dist;
        }
    }, { passive: true });
}
