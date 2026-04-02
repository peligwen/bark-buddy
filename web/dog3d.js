// 3D Dog Visualization — URDF-accurate articulated MechDog
// Driven by IMU telemetry, motion state, and sim joint angles when available.
//
// URDF reference (mechdog.urdf):
//   Body: 150mm x 85mm x 35mm
//   Hip spacing: 130mm longitudinal, 80mm lateral
//   Upper leg: 55mm cylinder, Lower leg: 60mm cylinder, Foot: 8mm sphere
//   Hip joint at body edge, -25mm below body center
//   Knee joint at bottom of upper leg (-55mm)
//   Foot at bottom of lower leg (-60mm)
//   Standing: hip=0.3rad, knee=-0.6rad

var Dog3D = (function () {
    "use strict";

    // --- URDF dimensions (meters, scaled 10x for visibility) ---
    var S = 10; // scale factor: 1 URDF meter = 10 scene units
    var BODY_L = 0.150 * S, BODY_W = 0.085 * S, BODY_H = 0.035 * S;
    var UPPER_LEN = 0.055 * S, UPPER_R = 0.008 * S;
    var LOWER_LEN = 0.060 * S, LOWER_R = 0.006 * S;
    var FOOT_R = 0.008 * S;
    var HIP_OFFSET_X = 0.065 * S;  // from body center
    var HIP_OFFSET_Z = 0.040 * S;  // lateral from body center (Y in URDF = Z in scene)
    var HIP_OFFSET_Y = -0.025 * S; // below body center

    var STAND_HIP = 0.3;   // radians
    var STAND_KNEE = -0.6;

    // Scene objects
    var scene, camera, renderer, dogGroup, ultraBeam;
    var container, animationId;

    // Camera orbit
    var isDragging = false, prevMouse = { x: 0, y: 0 };
    var cameraAngle = { theta: Math.PI / 5, phi: Math.PI / 5, radius: 3.5 };

    // Telemetry targets (smoothed)
    var targetPitch = 0, targetRoll = 0;
    var currentPitch = 0, currentRoll = 0;
    var currentMotion = "stop";
    var currentAction = null;
    var ultraDistance = null;
    var walkPhase = 0;
    var isFallen = false;

    // Leg pivots — each leg has { hipPivot, kneePivot, upperMesh, lowerMesh, footMesh }
    var legs = {};

    // Colors matching the UI theme
    var COL = {
        body:    0x2a2a4a,
        accent:  0xe94560,
        leg:     0x1a1a3e,
        head:    0x333366,
        eye:     0x4ecca3,
        ground:  0x0f0f2a,
        sensor:  0x666688,
        beam:    0x4ecca3,
    };

    function init(containerId) {
        container = document.getElementById(containerId);
        if (!container) return;

        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x0a0a1a);
        scene.fog = new THREE.Fog(0x0a0a1a, 12, 30);

        camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 50);
        updateCameraPosition();

        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        container.appendChild(renderer.domElement);

        // Lights
        scene.add(new THREE.AmbientLight(0x404060, 0.6));
        var dir = new THREE.DirectionalLight(0xffffff, 0.8);
        dir.position.set(3, 5, 4);
        dir.castShadow = true;
        dir.shadow.mapSize.set(1024, 1024);
        scene.add(dir);
        var rim = new THREE.DirectionalLight(0x4ecca3, 0.3);
        rim.position.set(-3, 2, -2);
        scene.add(rim);

        // Ground + grid
        var ground = new THREE.Mesh(
            new THREE.PlaneGeometry(30, 30),
            new THREE.MeshStandardMaterial({ color: COL.ground, roughness: 0.9 })
        );
        ground.rotation.x = -Math.PI / 2;
        ground.position.y = -0.01;
        ground.receiveShadow = true;
        scene.add(ground);
        scene.add(new THREE.GridHelper(15, 30, 0x1a1a3e, 0x111133));

        // Build articulated dog
        dogGroup = new THREE.Group();
        buildDog(dogGroup);
        // Position dog so feet are near ground level
        dogGroup.position.y = standingHeight();
        scene.add(dogGroup);

        setupControls();
        window.addEventListener("resize", onResize);
        animate();
    }

    function mat(color, opts) {
        var m = new THREE.MeshStandardMaterial({ color: color, roughness: 0.6, metalness: 0.2 });
        if (opts) {
            if (opts.emissive) { m.emissive = new THREE.Color(opts.emissive); m.emissiveIntensity = opts.ei || 0.8; }
            if (opts.roughness != null) m.roughness = opts.roughness;
            if (opts.metalness != null) m.metalness = opts.metalness;
        }
        return m;
    }

    function standingHeight() {
        // Calculate how high the body center needs to be for feet to touch ground
        // At standing pose: foot Y relative to hip = -upper*cos(hip) - lower*cos(hip+|knee|)
        var hipA = STAND_HIP, kneeA = STAND_KNEE;
        var footDy = UPPER_LEN * Math.cos(hipA) + LOWER_LEN * Math.cos(hipA + Math.abs(kneeA));
        return footDy - HIP_OFFSET_Y + FOOT_R;
    }

    function buildDog(group) {
        var bodyMat = mat(COL.body);
        var legMat = mat(COL.leg, { roughness: 0.7, metalness: 0.1 });
        var headMat = mat(COL.head, { roughness: 0.5 });
        var accentMat = mat(COL.accent, { roughness: 0.4, metalness: 0.3 });
        var eyeMat = mat(COL.eye, { emissive: COL.eye });
        var sensorMat = mat(COL.sensor, { metalness: 0.5, roughness: 0.3 });

        // Body chassis
        var body = new THREE.Mesh(new THREE.BoxGeometry(BODY_L, BODY_H, BODY_W), bodyMat);
        body.castShadow = true;
        group.add(body);

        // Top accent plate
        var plate = new THREE.Mesh(new THREE.BoxGeometry(BODY_L * 0.8, BODY_H * 0.15, BODY_W * 0.7), accentMat);
        plate.position.y = BODY_H * 0.55;
        group.add(plate);

        // Head block
        var headW = BODY_W * 0.65;
        var headH = BODY_H * 1.0;
        var headL = BODY_L * 0.28;
        var head = new THREE.Mesh(new THREE.BoxGeometry(headL, headH, headW), headMat);
        head.position.set(BODY_L / 2 + headL * 0.35, BODY_H * 0.3, 0);
        head.castShadow = true;
        group.add(head);

        // Snout
        var snout = new THREE.Mesh(new THREE.BoxGeometry(headL * 0.6, headH * 0.6, headW * 0.7), bodyMat);
        snout.position.set(BODY_L / 2 + headL * 0.85, BODY_H * 0.1, 0);
        group.add(snout);

        // Eyes
        var eyeR = 0.04 * S * 0.5;
        var eyeGeo = new THREE.SphereGeometry(eyeR, 8, 8);
        [-1, 1].forEach(function (side) {
            var eye = new THREE.Mesh(eyeGeo, eyeMat);
            eye.position.set(BODY_L / 2 + headL * 0.6, BODY_H * 0.55, side * headW * 0.35);
            group.add(eye);
        });

        // Ears
        [-1, 1].forEach(function (side) {
            var ear = new THREE.Mesh(new THREE.BoxGeometry(headL * 0.3, headH * 0.5, headW * 0.2), accentMat);
            ear.position.set(BODY_L / 2 + headL * 0.15, BODY_H * 0.85, side * headW * 0.42);
            ear.rotation.z = 0.2;
            group.add(ear);
        });

        // Tail nub
        var tail = new THREE.Mesh(new THREE.BoxGeometry(BODY_L * 0.12, BODY_H * 0.4, BODY_H * 0.4), accentMat);
        tail.position.set(-BODY_L / 2 - BODY_L * 0.06, BODY_H * 0.3, 0);
        tail.rotation.z = -0.3;
        group.add(tail);

        // Ultrasonic sensors (two small cylinders on front face)
        var usGeo = new THREE.CylinderGeometry(0.02 * S, 0.02 * S, 0.02 * S, 8);
        [-1, 1].forEach(function (side) {
            var us = new THREE.Mesh(usGeo, sensorMat);
            us.position.set(BODY_L / 2 + headL * 1.05, BODY_H * 0.1, side * 0.01 * S);
            us.rotation.z = Math.PI / 2;
            group.add(us);
        });

        // Ultrasonic beam (cone, hidden until distance data arrives)
        // ConeGeometry tip at +Y, base at -Y. We rotate so tip stays at
        // the sensor (maps to -X after rotation) and the wide base extends
        // forward (+X). Then we offset +X by half the cone height so the
        // tip sits at the sensor origin.
        var beamGeo = new THREE.ConeGeometry(0.15 * S, 1.0 * S, 8, 1, true);
        var beamMat = new THREE.MeshBasicMaterial({
            color: COL.beam, transparent: true, opacity: 0.12,
            side: THREE.DoubleSide, depthWrite: false,
        });
        ultraBeam = new THREE.Mesh(beamGeo, beamMat);
        ultraBeam.rotation.z = Math.PI / 2; // tip → -X (sensor), base → +X (forward)
        ultraBeam.visible = false;
        // Store sensor mount point for updateUltraBeam
        ultraBeam.userData.sensorX = BODY_L / 2 + headL * 1.05;
        ultraBeam.userData.sensorY = BODY_H * 0.1;
        ultraBeam.position.set(ultraBeam.userData.sensorX + 0.5 * S, ultraBeam.userData.sensorY, 0);
        group.add(ultraBeam);

        // --- Articulated legs ---
        var legDefs = [
            { name: "fl", x:  HIP_OFFSET_X, z:  HIP_OFFSET_Z },
            { name: "fr", x:  HIP_OFFSET_X, z: -HIP_OFFSET_Z },
            { name: "rl", x: -HIP_OFFSET_X, z:  HIP_OFFSET_Z },
            { name: "rr", x: -HIP_OFFSET_X, z: -HIP_OFFSET_Z },
        ];

        legDefs.forEach(function (def) {
            var leg = buildLeg(def.name, legMat, accentMat);
            // Position hip pivot at URDF joint origin relative to body center
            leg.hipPivot.position.set(def.x, HIP_OFFSET_Y, def.z);
            group.add(leg.hipPivot);
            legs[def.name] = leg;
        });

        // Set initial standing pose
        setAllLegs(STAND_HIP, STAND_KNEE);
    }

    function buildLeg(name, legMat, footMat) {
        // Hip pivot — rotates around Z axis (pitch axis in scene = Y axis in URDF)
        var hipPivot = new THREE.Group();

        // Upper leg mesh — cylinder centered at half its length below the pivot
        var upperGeo = new THREE.CylinderGeometry(UPPER_R, UPPER_R, UPPER_LEN, 8);
        var upperMesh = new THREE.Mesh(upperGeo, legMat);
        upperMesh.position.y = -UPPER_LEN / 2;
        upperMesh.castShadow = true;
        hipPivot.add(upperMesh);

        // Joint sphere at hip
        var jointGeo = new THREE.SphereGeometry(UPPER_R * 1.5, 8, 8);
        var jointMesh = new THREE.Mesh(jointGeo, footMat);
        hipPivot.add(jointMesh);

        // Knee pivot — at bottom of upper leg
        var kneePivot = new THREE.Group();
        kneePivot.position.y = -UPPER_LEN;
        hipPivot.add(kneePivot);

        // Knee joint sphere
        var kneeJoint = new THREE.Mesh(jointGeo, footMat);
        kneePivot.add(kneeJoint);

        // Lower leg mesh
        var lowerGeo = new THREE.CylinderGeometry(LOWER_R, LOWER_R, LOWER_LEN, 8);
        var lowerMesh = new THREE.Mesh(lowerGeo, legMat);
        lowerMesh.position.y = -LOWER_LEN / 2;
        lowerMesh.castShadow = true;
        kneePivot.add(lowerMesh);

        // Foot sphere at bottom of lower leg
        var footGeo = new THREE.SphereGeometry(FOOT_R, 8, 8);
        var footMesh = new THREE.Mesh(footGeo, footMat);
        footMesh.position.y = -LOWER_LEN;
        footMesh.castShadow = true;
        kneePivot.add(footMesh);

        return {
            hipPivot: hipPivot,
            kneePivot: kneePivot,
            upperMesh: upperMesh,
            lowerMesh: lowerMesh,
            footMesh: footMesh,
        };
    }

    function setLeg(name, hipAngle, kneeAngle) {
        var leg = legs[name];
        if (!leg) return;
        // URDF: hip rotates around Y axis (pitch). In Three.js scene, legs hang down (-Y),
        // and hip pitch rotates around Z (since URDF X=scene X forward, URDF Y=scene Z, URDF Z=scene Y up)
        // Actually: URDF axis is (0,1,0) = Y. In our scene, the leg hangs -Y and the URDF Y axis
        // maps to our scene Z axis. So hip rotation = rotation around scene Z.
        leg.hipPivot.rotation.z = hipAngle;
        leg.kneePivot.rotation.z = kneeAngle;
    }

    function setAllLegs(hip, knee) {
        ["fl", "fr", "rl", "rr"].forEach(function (name) {
            setLeg(name, hip, knee);
        });
    }

    // --- Gait animation (when no sim joint data) ---

    function animateGait(dt) {
        var speed = 0;
        if (currentMotion === "forward") speed = 6;
        else if (currentMotion === "backward") speed = -4;
        else if (currentMotion === "left" || currentMotion === "right") speed = 3;

        if (speed === 0 && currentAction == null) {
            // Blend back to standing
            walkPhase = 0;
            setAllLegs(STAND_HIP, STAND_KNEE);
            return;
        }

        if (currentAction != null) {
            animateAction();
            return;
        }

        walkPhase += speed * dt;

        // Trot gait: diagonal pairs move together
        // FL+RR are one pair, FR+RL are the other
        var amp = 0.25; // hip swing amplitude (radians)
        var kneeAmp = 0.2;

        var phA = Math.sin(walkPhase);
        var phB = Math.sin(walkPhase + Math.PI);

        setLeg("fl", STAND_HIP + amp * phA, STAND_KNEE - kneeAmp * Math.max(0, phA));
        setLeg("rr", STAND_HIP + amp * phA, STAND_KNEE - kneeAmp * Math.max(0, phA));
        setLeg("fr", STAND_HIP + amp * phB, STAND_KNEE - kneeAmp * Math.max(0, phB));
        setLeg("rl", STAND_HIP + amp * phB, STAND_KNEE - kneeAmp * Math.max(0, phB));
    }

    function animateAction() {
        // Simplified action poses
        var a = currentAction;
        if (a === 1) {
            // Wave: front-left leg raised
            setLeg("fl", -0.8, -1.2);
            setLeg("fr", STAND_HIP, STAND_KNEE);
            setLeg("rl", STAND_HIP * 1.3, STAND_KNEE * 0.8);
            setLeg("rr", STAND_HIP * 1.3, STAND_KNEE * 0.8);
        } else if (a === 4) {
            // Sit: front legs straight, back legs folded
            setLeg("fl", 0.05, -0.1);
            setLeg("fr", 0.05, -0.1);
            setLeg("rl", 1.0, -1.4);
            setLeg("rr", 1.0, -1.4);
        } else if (a === 5) {
            // Lie down: all legs folded flat
            setLeg("fl", 0.8, -1.2);
            setLeg("fr", 0.8, -1.2);
            setLeg("rl", 0.8, -1.2);
            setLeg("rr", 0.8, -1.2);
        } else {
            setAllLegs(STAND_HIP, STAND_KNEE);
        }
    }

    // --- Sim joint data ---
    var simJoints = null;

    function applySimJoints() {
        if (!simJoints) return;
        ["fl", "fr", "rl", "rr"].forEach(function (name) {
            var j = simJoints[name];
            if (j) {
                setLeg(name, j.hip, j.knee);
            }
        });
    }

    // --- Ultrasonic beam ---

    function updateUltraBeam() {
        if (ultraDistance == null || ultraDistance > 2500) {
            if (ultraBeam) ultraBeam.visible = false;
            return;
        }
        if (!ultraBeam) return;
        ultraBeam.visible = true;

        // Convert mm to scene units, clamp
        var len = (ultraDistance / 1000) * S;
        var maxLen = 3.0 * S;
        len = Math.min(len, maxLen);

        // Scale the cone (base geometry is 1.0*S tall)
        var lengthScale = len / (1.0 * S);
        ultraBeam.scale.y = lengthScale;
        var widthScale = 0.5 + 0.5 * (len / maxLen);
        ultraBeam.scale.x = widthScale;
        ultraBeam.scale.z = widthScale;

        // Anchor tip at sensor mount, offset center forward by half scaled length
        var sx = ultraBeam.userData.sensorX;
        var sy = ultraBeam.userData.sensorY;
        ultraBeam.position.set(sx + (len / 2), sy, 0);

        // Color by distance
        if (ultraDistance < 150) {
            ultraBeam.material.color.setHex(0xe94560);
            ultraBeam.material.opacity = 0.25;
        } else if (ultraDistance < 400) {
            ultraBeam.material.color.setHex(0xf0a500);
            ultraBeam.material.opacity = 0.18;
        } else {
            ultraBeam.material.color.setHex(COL.beam);
            ultraBeam.material.opacity = 0.12;
        }
    }

    // --- Camera ---

    function updateCameraPosition() {
        camera.position.x = cameraAngle.radius * Math.cos(cameraAngle.phi) * Math.sin(cameraAngle.theta);
        camera.position.y = cameraAngle.radius * Math.sin(cameraAngle.phi);
        camera.position.z = cameraAngle.radius * Math.cos(cameraAngle.phi) * Math.cos(cameraAngle.theta);
        camera.lookAt(0, standingHeight() * 0.5, 0);
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

        el.addEventListener("pointerup", function () { isDragging = false; });

        el.addEventListener("wheel", function (e) {
            e.preventDefault();
            cameraAngle.radius = Math.max(1.5, Math.min(10, cameraAngle.radius + e.deltaY * 0.005));
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
                cameraAngle.radius = Math.max(1.5, Math.min(10, cameraAngle.radius + (lastTouchDist - dist) * 0.02));
                updateCameraPosition();
                lastTouchDist = dist;
            }
        }, { passive: true });
    }

    // --- Animation loop ---

    var lastTime = 0;

    function animate(time) {
        animationId = requestAnimationFrame(animate);
        var dt = lastTime ? Math.min((time - lastTime) / 1000, 0.1) : 0.016;
        lastTime = time;

        // Smooth IMU interpolation
        currentPitch += (targetPitch - currentPitch) * 0.15;
        currentRoll += (targetRoll - currentRoll) * 0.15;

        if (dogGroup) {
            // Pitch = forward/back tilt around Z, Roll = side tilt around X
            dogGroup.rotation.z = currentPitch * (Math.PI / 180);
            dogGroup.rotation.x = currentRoll * (Math.PI / 180);

            // Leg animation: use sim joints if available, otherwise gait from motion state
            if (simJoints) {
                applySimJoints();
            } else {
                animateGait(dt);
            }

            updateUltraBeam();
        }

        renderer.render(scene, camera);
    }

    function onResize() {
        if (!container || !camera || !renderer) return;
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    }

    // --- Public API ---

    return {
        init: init,

        updateIMU: function (msg) {
            targetPitch = msg.pitch;
            targetRoll = msg.roll;
            if (msg.motion != null) currentMotion = msg.motion;
            if (msg.action !== undefined) currentAction = msg.action;
            if (msg.joints) simJoints = msg.joints;
        },

        updateUltrasonic: function (distance_mm) {
            ultraDistance = distance_mm;
        },

        setFallen: function (fallen) {
            isFallen = fallen;
            if (!dogGroup) return;
            dogGroup.traverse(function (child) {
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
})();
