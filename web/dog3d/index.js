// Main entry point — assembles submodules and exports the Dog3D public API.
import { state, S, COL, BODY_L, BODY_H, standingHeight } from './state.js';
import { buildDog } from './model.js';
import { animateGait, setPose, clearPose, getPoseNames, applySimJoints } from './gait.js';
import { initUltraHit, updateUltraBeam } from './sonar.js';
import { clearWalls, buildWallsFromSegments } from './walls.js';
import { toggleOverlay, updateOverlay } from './overlay.js';
import { updateCameraPosition, setupControls } from './camera.js';

var lastTime = 0;

function init(containerId) {
    state.container = document.getElementById(containerId);
    if (!state.container) return;

    // Defer init if the container has no dimensions yet (flex layout not computed)
    if (state.container.clientWidth === 0 || state.container.clientHeight === 0) {
        requestAnimationFrame(function () { init(containerId); });
        return;
    }

    state.scene = new THREE.Scene();
    state.scene.background = new THREE.Color(COL.sceneBg);
    state.scene.fog = new THREE.Fog(COL.sceneBg, 20, 40);

    state.camera = new THREE.PerspectiveCamera(
        45, state.container.clientWidth / state.container.clientHeight, 0.1, 50);
    updateCameraPosition();

    state.renderer = new THREE.WebGLRenderer({ antialias: true });
    state.renderer.setSize(state.container.clientWidth, state.container.clientHeight);
    state.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    state.renderer.shadowMap.enabled = true;
    state.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    state.renderer.toneMapping = THREE.LinearToneMapping;
    state.renderer.toneMappingExposure = 1.0;
    state.container.appendChild(state.renderer.domElement);

    // Ambient fill — slightly dimmer for dark theme
    state.scene.add(new THREE.AmbientLight(0xffffff, 0.5));

    // Overhead key light
    var overhead = new THREE.DirectionalLight(0xffffff, 0.9);
    overhead.position.set(0.5, 8, 0.5);
    overhead.castShadow = true;
    overhead.shadow.mapSize.set(1024, 1024);
    overhead.shadow.camera.near = 1;
    overhead.shadow.camera.far = 15;
    overhead.shadow.camera.left = -4;
    overhead.shadow.camera.right = 4;
    overhead.shadow.camera.top = 4;
    overhead.shadow.camera.bottom = -4;
    state.scene.add(overhead);

    // Secondary fill from the side — softer
    var fill = new THREE.DirectionalLight(0xffffff, 0.3);
    fill.position.set(-3, 4, 2);
    state.scene.add(fill);

    // Dark floor
    var ground = new THREE.Mesh(
        new THREE.PlaneGeometry(30, 30),
        new THREE.MeshStandardMaterial({ color: COL.ground, roughness: 0.8, metalness: 0.0 })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.01;
    ground.receiveShadow = true;
    state.scene.add(ground);

    // Subtle dark grid
    state.scene.add(new THREE.GridHelper(15, 30, COL.grid, 0x22262e));

    // Build articulated dog
    state.dogGroup = new THREE.Group();
    state.dogGroup.rotation.order = 'YZX'; // Yaw first (Y), then pitch (Z), then roll (X)
    buildDog(state.dogGroup);
    // Position dog so feet are near ground level
    state.dogGroup.position.y = standingHeight();
    state.scene.add(state.dogGroup);

    initUltraHit();
    setupControls();
    window.addEventListener("resize", onResize);
    animate();
}

function animate(time) {
    state.animationId = requestAnimationFrame(animate);
    var dt = lastTime ? Math.min((time - lastTime) / 1000, 0.1) : 0.016;
    lastTime = time;

    // Smooth interpolation
    state.currentPitch += (state.targetPitch - state.currentPitch) * 0.15;
    state.currentRoll += (state.targetRoll - state.currentRoll) * 0.15;
    state.currentX += (state.targetX - state.currentX) * 0.15;
    state.currentZ += (state.targetZ - state.currentZ) * 0.15;
    // Smooth yaw with wrapping
    var yawDiff = state.targetYaw - state.currentYaw;
    if (yawDiff > Math.PI) yawDiff -= 2 * Math.PI;
    if (yawDiff < -Math.PI) yawDiff += 2 * Math.PI;
    state.currentYaw += yawDiff * 0.15;

    if (state.dogGroup) {
        // Position: sim X = scene X, sim Y = scene Z
        state.dogGroup.position.x = state.currentX;
        state.dogGroup.position.y = standingHeight() + state.bodyBounce;
        state.dogGroup.position.z = state.currentZ;

        // Yaw rotation around Y axis, then pitch/roll on top
        state.dogGroup.rotation.y = state.currentYaw;
        state.dogGroup.rotation.z = state.currentPitch * (Math.PI / 180);
        state.dogGroup.rotation.x = state.currentRoll * (Math.PI / 180);

        // Leg animation: use sim joints if available, otherwise gait from motion state
        if (state.simJoints) {
            applySimJoints();
            state.bodyBounce = 0;
        } else {
            animateGait(dt);
        }

        updateUltraBeam();
    }

    updateOverlay();
    updateCameraPosition();
    state.renderer.render(state.scene, state.camera);
}

function onResize() {
    if (!state.container || !state.camera || !state.renderer) return;
    state.camera.aspect = state.container.clientWidth / state.container.clientHeight;
    state.camera.updateProjectionMatrix();
    state.renderer.setSize(state.container.clientWidth, state.container.clientHeight);
}

// --- Public API ---
var Dog3D = {
    init: init,

    debug: function () {
        return {
            targetX: state.targetX, targetZ: state.targetZ, targetYaw: state.targetYaw,
            currentX: state.currentX, currentZ: state.currentZ, currentYaw: state.currentYaw,
            headingDeg: state.targetYaw * (180 / Math.PI),
            motion: state.currentMotion,
            ultraDistance: state.ultraDistance,
        };
    },

    updateIMU: function (msg) {
        state.targetPitch = msg.pitch;
        state.targetRoll = msg.roll;
        if (msg.joints) state.simJoints = msg.joints;
    },

    updateOdometry: function (msg) {
        if (msg.motion != null) {
            state.currentMotion = msg.motion;
            // Movement clears any held pose
            if (msg.motion !== "stop") clearPose();
        }
        if (msg.x != null && msg.y != null) {
            state.targetX = msg.x * S;
            state.targetZ = msg.y * S;
        }
        // rotation.y = +90deg faces -Z, but heading=90deg (turned left) should
        // face +Z (sim +Y), so negate to get correct visual direction
        if (msg.heading != null) {
            state.targetYaw = -msg.heading * (Math.PI / 180);
        }
    },

    updateUltrasonic: function (distance_mm) {
        state.ultraDistance = distance_mm;
    },

    reset: function () {
        state.targetPitch = state.targetRoll = 0;
        state.currentPitch = state.currentRoll = 0;
        state.targetX = state.targetZ = state.targetYaw = 0;
        state.currentX = state.currentZ = state.currentYaw = 0;
        state.camTargetX = state.camTargetZ = 0;
        state.currentMotion = "stop";
        state.currentAction = null;
        state.walkPhase = 0;
        state.simJoints = null;
        clearWalls();
        if (state.dogGroup) {
            state.dogGroup.position.set(0, standingHeight(), 0);
            state.dogGroup.rotation.set(0, 0, 0);
        }
    },

    setMapData: function (data) {
        if (data && data.walls) {
            buildWallsFromSegments(data.walls);
        }
    },

    clearScanWalls: function () {
        clearWalls();
    },

    toggleOverlay: function (show) {
        toggleOverlay(show);
    },

    setPose: function (name) {
        setPose(name);
    },

    getPoseNames: function () {
        return getPoseNames();
    },

    setFallen: function (fallen) {
        state.isFallen = fallen;
        if (!state.dogGroup) return;
        state.dogGroup.traverse(function (child) {
            if (child.isMesh && child.material.emissive) {
                if (fallen) {
                    child.material.emissive.setHex(0x330000);
                    child.material.emissiveIntensity = 0.5;
                } else {
                    var isEye = child.material.color.getHex() === COL.eye;
                    child.material.emissive.setHex(isEye ? COL.eye : 0x000000);
                    child.material.emissiveIntensity = isEye ? 0.8 : 0;
                }
            }
        });
    },
};

export default Dog3D;
