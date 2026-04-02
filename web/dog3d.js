// 3D Dog Visualization — Three.js scene driven by IMU telemetry
// Renders a geometric MechDog model that tilts with real pitch/roll data.

var Dog3D = (function () {
    "use strict";

    var scene, camera, renderer, dogGroup, gridHelper;
    var targetPitch = 0, targetRoll = 0;
    var currentPitch = 0, currentRoll = 0;
    var container;
    var animationId;
    var isDragging = false, prevMouse = { x: 0, y: 0 };
    var cameraAngle = { theta: Math.PI / 6, phi: Math.PI / 4, radius: 4 };

    var BODY_COLOR = 0x2a2a4a;
    var ACCENT_COLOR = 0xe94560;
    var LEG_COLOR = 0x1a1a3e;
    var HEAD_COLOR = 0x333366;
    var EYE_COLOR = 0x4ecca3;
    var GROUND_COLOR = 0x0f0f2a;

    function init(containerId) {
        container = document.getElementById(containerId);
        if (!container) return;

        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x0a0a1a);
        scene.fog = new THREE.Fog(0x0a0a1a, 8, 20);

        // Camera
        camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 50);
        updateCameraPosition();

        // Renderer
        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        container.appendChild(renderer.domElement);

        // Lights
        var ambient = new THREE.AmbientLight(0x404060, 0.6);
        scene.add(ambient);

        var dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(3, 5, 4);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 1024;
        dirLight.shadow.mapSize.height = 1024;
        dirLight.shadow.camera.near = 0.5;
        dirLight.shadow.camera.far = 20;
        dirLight.shadow.camera.left = -5;
        dirLight.shadow.camera.right = 5;
        dirLight.shadow.camera.top = 5;
        dirLight.shadow.camera.bottom = -5;
        scene.add(dirLight);

        var rimLight = new THREE.DirectionalLight(0x4ecca3, 0.3);
        rimLight.position.set(-3, 2, -2);
        scene.add(rimLight);

        // Ground plane
        var groundGeo = new THREE.PlaneGeometry(20, 20);
        var groundMat = new THREE.MeshStandardMaterial({
            color: GROUND_COLOR,
            roughness: 0.9,
            metalness: 0.1
        });
        var ground = new THREE.Mesh(groundGeo, groundMat);
        ground.rotation.x = -Math.PI / 2;
        ground.position.y = -0.01;
        ground.receiveShadow = true;
        scene.add(ground);

        // Grid
        gridHelper = new THREE.GridHelper(10, 20, 0x1a1a3e, 0x111133);
        scene.add(gridHelper);

        // Build dog
        dogGroup = new THREE.Group();
        buildDog(dogGroup);
        scene.add(dogGroup);

        // Events
        setupControls();
        window.addEventListener("resize", onResize);

        animate();
    }

    function buildDog(group) {
        var bodyMat = new THREE.MeshStandardMaterial({ color: BODY_COLOR, roughness: 0.6, metalness: 0.2 });
        var legMat = new THREE.MeshStandardMaterial({ color: LEG_COLOR, roughness: 0.7, metalness: 0.1 });
        var headMat = new THREE.MeshStandardMaterial({ color: HEAD_COLOR, roughness: 0.5, metalness: 0.2 });
        var accentMat = new THREE.MeshStandardMaterial({ color: ACCENT_COLOR, roughness: 0.4, metalness: 0.3 });
        var eyeMat = new THREE.MeshStandardMaterial({ color: EYE_COLOR, emissive: EYE_COLOR, emissiveIntensity: 0.8 });

        // Body — main chassis
        var body = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.35, 0.7), bodyMat);
        body.position.y = 0.9;
        body.castShadow = true;
        group.add(body);

        // Top plate accent
        var topPlate = new THREE.Mesh(new THREE.BoxGeometry(1.2, 0.05, 0.5), accentMat);
        topPlate.position.y = 1.1;
        topPlate.castShadow = true;
        group.add(topPlate);

        // Head
        var head = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.3, 0.5), headMat);
        head.position.set(0.95, 1.05, 0);
        head.castShadow = true;
        group.add(head);

        // Snout
        var snout = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.15, 0.3), bodyMat);
        snout.position.set(1.2, 0.97, 0);
        snout.castShadow = true;
        group.add(snout);

        // Eyes
        var eyeGeo = new THREE.SphereGeometry(0.04, 8, 8);
        var eyeL = new THREE.Mesh(eyeGeo, eyeMat);
        eyeL.position.set(1.15, 1.1, 0.18);
        group.add(eyeL);
        var eyeR = new THREE.Mesh(eyeGeo, eyeMat);
        eyeR.position.set(1.15, 1.1, -0.18);
        group.add(eyeR);

        // Ears
        var earGeo = new THREE.BoxGeometry(0.08, 0.15, 0.12);
        var earL = new THREE.Mesh(earGeo, accentMat);
        earL.position.set(0.9, 1.27, 0.22);
        earL.rotation.z = 0.2;
        group.add(earL);
        var earR = new THREE.Mesh(earGeo, accentMat);
        earR.position.set(0.9, 1.27, -0.22);
        earR.rotation.z = 0.2;
        group.add(earR);

        // Legs (4 legs with upper + lower segments)
        var legPositions = [
            { x: 0.55, z: 0.3 },   // front-left
            { x: 0.55, z: -0.3 },  // front-right
            { x: -0.55, z: 0.3 },  // back-left
            { x: -0.55, z: -0.3 }, // back-right
        ];

        legPositions.forEach(function (pos) {
            // Upper leg (hip joint area)
            var upper = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.35, 0.12), legMat);
            upper.position.set(pos.x, 0.62, pos.z);
            upper.castShadow = true;
            group.add(upper);

            // Lower leg
            var lower = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.35, 0.1), legMat);
            lower.position.set(pos.x, 0.27, pos.z);
            lower.castShadow = true;
            group.add(lower);

            // Foot pad
            var foot = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.06, 0.14), accentMat);
            foot.position.set(pos.x, 0.07, pos.z);
            foot.castShadow = true;
            group.add(foot);
        });

        // Tail — small nub
        var tail = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.08, 0.08), accentMat);
        tail.position.set(-0.9, 1.0, 0);
        tail.rotation.z = -0.3;
        group.add(tail);

        // Ultrasonic sensor (front, two small cylinders)
        var sensorGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.04, 8);
        var sensorMat = new THREE.MeshStandardMaterial({ color: 0x666688, metalness: 0.5, roughness: 0.3 });
        var sensorL = new THREE.Mesh(sensorGeo, sensorMat);
        sensorL.position.set(1.32, 0.97, 0.08);
        sensorL.rotation.z = Math.PI / 2;
        group.add(sensorL);
        var sensorR = new THREE.Mesh(sensorGeo, sensorMat);
        sensorR.position.set(1.32, 0.97, -0.08);
        sensorR.rotation.z = Math.PI / 2;
        group.add(sensorR);
    }

    function updateCameraPosition() {
        camera.position.x = cameraAngle.radius * Math.cos(cameraAngle.phi) * Math.sin(cameraAngle.theta);
        camera.position.y = cameraAngle.radius * Math.sin(cameraAngle.phi);
        camera.position.z = cameraAngle.radius * Math.cos(cameraAngle.phi) * Math.cos(cameraAngle.theta);
        camera.lookAt(0, 0.7, 0);
    }

    function setupControls() {
        var el = renderer.domElement;

        el.addEventListener("pointerdown", function (e) {
            isDragging = true;
            prevMouse.x = e.clientX;
            prevMouse.y = e.clientY;
            el.setPointerCapture(e.pointerId);
        });

        el.addEventListener("pointermove", function (e) {
            if (!isDragging) return;
            var dx = e.clientX - prevMouse.x;
            var dy = e.clientY - prevMouse.y;
            prevMouse.x = e.clientX;
            prevMouse.y = e.clientY;

            cameraAngle.theta -= dx * 0.008;
            cameraAngle.phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, cameraAngle.phi + dy * 0.008));
            updateCameraPosition();
        });

        el.addEventListener("pointerup", function (e) {
            isDragging = false;
        });

        el.addEventListener("wheel", function (e) {
            e.preventDefault();
            cameraAngle.radius = Math.max(2, Math.min(10, cameraAngle.radius + e.deltaY * 0.005));
            updateCameraPosition();
        }, { passive: false });

        // Touch zoom
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
                var delta = lastTouchDist - dist;
                cameraAngle.radius = Math.max(2, Math.min(10, cameraAngle.radius + delta * 0.02));
                updateCameraPosition();
                lastTouchDist = dist;
            }
        }, { passive: true });
    }

    function animate() {
        animationId = requestAnimationFrame(animate);

        // Smooth interpolation toward target angles
        currentPitch += (targetPitch - currentPitch) * 0.15;
        currentRoll += (targetRoll - currentRoll) * 0.15;

        if (dogGroup) {
            // Pitch = rotation around Z axis (nose up/down)
            // Roll = rotation around X axis (side tilt)
            dogGroup.rotation.z = currentPitch * (Math.PI / 180);
            dogGroup.rotation.x = currentRoll * (Math.PI / 180);
        }

        renderer.render(scene, camera);
    }

    function onResize() {
        if (!container || !camera || !renderer) return;
        var w = container.clientWidth;
        var h = container.clientHeight;
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(w, h);
    }

    // Public API
    return {
        init: init,

        updateIMU: function (pitch, roll) {
            targetPitch = pitch;
            targetRoll = roll;
        },

        setFallen: function (fallen) {
            if (!dogGroup) return;
            // Visual indicator: tint red on fall
            dogGroup.traverse(function (child) {
                if (child.isMesh && child.material.emissive) {
                    if (fallen) {
                        child.material.emissive.setHex(0x330000);
                        child.material.emissiveIntensity = 0.5;
                    } else {
                        child.material.emissive.setHex(
                            child.material.color.getHex() === EYE_COLOR ? EYE_COLOR : 0x000000
                        );
                        child.material.emissiveIntensity = child.material.color.getHex() === EYE_COLOR ? 0.8 : 0;
                    }
                }
            });
        }
    };
})();
