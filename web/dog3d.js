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
    // Chase camera: theta is offset from dog's yaw (0 = directly behind)
    var cameraAngle = { thetaOffset: 0, phi: Math.PI / 5, radius: 8 };
    var camYaw = 0; // smoothed camera yaw tracking the dog

    // Telemetry targets (smoothed)
    var targetPitch = 0, targetRoll = 0;
    var currentPitch = 0, currentRoll = 0;
    var targetX = 0, targetZ = 0, targetYaw = 0;
    var currentX = 0, currentZ = 0, currentYaw = 0;
    var currentMotion = "stop";
    var currentAction = null;
    var ultraDistance = null;
    var walkPhase = 0;
    var isFallen = false;

    // Leg pivots — each leg has { hipPivot, kneePivot, upperMesh, lowerMesh, footMesh }
    var legs = {};

    // Industrial yellow dog on dark floor
    var COL = {
        body:    0xd4a017,  // safety yellow
        accent:  0x1a1a1a,  // black trim
        leg:     0x2c2c2c,  // dark charcoal legs
        head:    0xe2b52e,  // lighter yellow head
        eye:     0x33ff66,  // bright green status LED
        ground:  0x1a1d24,  // dark floor matching UI
        sensor:  0x888888,  // silver sensor housings
        beam:    0x33aaff,  // blue ultrasonic beam
        joint:   0x444444,  // dark joint spheres
        grid:    0x2a2f3a,  // subtle dark grid
        sceneBg: 0x1e222b,  // scene background matching card bg
    };

    function init(containerId) {
        container = document.getElementById(containerId);
        if (!container) return;

        // Defer init if the container has no dimensions yet (flex layout not computed)
        if (container.clientWidth === 0 || container.clientHeight === 0) {
            requestAnimationFrame(function () { init(containerId); });
            return;
        }

        scene = new THREE.Scene();
        scene.background = new THREE.Color(COL.sceneBg);
        scene.fog = new THREE.Fog(COL.sceneBg, 20, 40);

        camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 50);
        updateCameraPosition();

        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        renderer.toneMapping = THREE.LinearToneMapping;
        renderer.toneMappingExposure = 1.0;
        container.appendChild(renderer.domElement);

        // Ambient fill — slightly dimmer for dark theme
        scene.add(new THREE.AmbientLight(0xffffff, 0.5));

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
        scene.add(overhead);

        // Secondary fill from the side — softer
        var fill = new THREE.DirectionalLight(0xffffff, 0.3);
        fill.position.set(-3, 4, 2);
        scene.add(fill);

        // Dark floor
        var ground = new THREE.Mesh(
            new THREE.PlaneGeometry(30, 30),
            new THREE.MeshStandardMaterial({ color: COL.ground, roughness: 0.8, metalness: 0.0 })
        );
        ground.rotation.x = -Math.PI / 2;
        ground.position.y = -0.01;
        ground.receiveShadow = true;
        scene.add(ground);

        // Subtle dark grid
        scene.add(new THREE.GridHelper(15, 30, COL.grid, 0x22262e));

        // Build articulated dog
        dogGroup = new THREE.Group();
        dogGroup.rotation.order = 'YZX'; // Yaw first (Y), then pitch (Z), then roll (X)
        buildDog(dogGroup);
        // Position dog so feet are near ground level
        dogGroup.position.y = standingHeight();
        scene.add(dogGroup);

        initUltraHit();
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
        var bodyMat = mat(COL.body, { roughness: 0.45, metalness: 0.1 });
        var legMat = mat(COL.leg, { roughness: 0.6, metalness: 0.15 });
        var headMat = mat(COL.head, { roughness: 0.4, metalness: 0.1 });
        var accentMat = mat(COL.accent, { roughness: 0.5, metalness: 0.2 });
        var eyeMat = mat(COL.eye, { emissive: COL.eye, ei: 0.5 });
        var sensorMat = mat(COL.sensor, { metalness: 0.4, roughness: 0.3 });

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

        // Eyes — small LED dots on head front
        var eyeR = 0.006 * S;
        var eyeGeo = new THREE.SphereGeometry(eyeR, 8, 8);
        [-1, 1].forEach(function (side) {
            var eye = new THREE.Mesh(eyeGeo, eyeMat);
            eye.position.set(BODY_L / 2 + headL * 0.6, BODY_H * 0.45, side * headW * 0.28);
            group.add(eye);
        });

        // Ears
        [-1, 1].forEach(function (side) {
            var ear = new THREE.Mesh(new THREE.BoxGeometry(headL * 0.3, headH * 0.5, headW * 0.2), accentMat);
            ear.position.set(BODY_L / 2 + headL * 0.15, BODY_H * 0.85, side * headW * 0.42);
            ear.rotation.z = 0.2;
            group.add(ear);
        });

        // Tail nub + red flag
        var tail = new THREE.Mesh(new THREE.BoxGeometry(BODY_L * 0.12, BODY_H * 0.4, BODY_H * 0.4), accentMat);
        tail.position.set(-BODY_L / 2 - BODY_L * 0.06, BODY_H * 0.3, 0);
        tail.rotation.z = -0.3;
        group.add(tail);

        var flagPole = new THREE.Mesh(
            new THREE.CylinderGeometry(0.003 * S, 0.003 * S, 0.08 * S, 6),
            mat(0x444444)
        );
        flagPole.position.set(-BODY_L / 2 - BODY_L * 0.1, BODY_H * 0.7, 0);
        group.add(flagPole);

        var flag = new THREE.Mesh(
            new THREE.PlaneGeometry(0.04 * S, 0.025 * S),
            new THREE.MeshBasicMaterial({ color: 0xdd2222, side: THREE.DoubleSide })
        );
        flag.position.set(-BODY_L / 2 - BODY_L * 0.1 - 0.02 * S, BODY_H * 0.95, 0);
        group.add(flag);

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

        var jointMat = mat(COL.joint, { roughness: 0.5, metalness: 0.3 });

        legDefs.forEach(function (def) {
            var leg = buildLeg(def.name, legMat, jointMat);
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

    var bodyBounce = 0;

    function animateGait(dt) {
        var speed = 0;
        if (currentMotion === "forward") speed = 6;
        else if (currentMotion === "backward") speed = -4;
        else if (currentMotion === "left" || currentMotion === "right") speed = 3;

        if (speed === 0) {
            walkPhase = 0;
            bodyBounce = 0;
            // Pose interpolation or action takes priority over standing reset
            if (targetPose || currentAction != null) {
                animateAction();
                return;
            }
            setAllLegs(STAND_HIP, STAND_KNEE);
            return;
        }

        if (currentAction != null || targetPose) {
            animateAction();
            bodyBounce = 0;
            return;
        }

        walkPhase += speed * dt;

        // Trot gait: diagonal pairs move together
        var hipAmp = 0.22;   // hip swing amplitude (radians)
        var kneeAmp = 0.35;  // knee lift amplitude
        var kneeBias = 0.1;  // extra knee bend to keep feet above ground

        var phA = Math.sin(walkPhase);
        var phB = Math.sin(walkPhase + Math.PI);

        // Knee lifts during swing (positive phase), extra bend during stance
        var kneeA = STAND_KNEE - kneeBias - kneeAmp * Math.abs(phA);
        var kneeB = STAND_KNEE - kneeBias - kneeAmp * Math.abs(phB);

        setLeg("fl", STAND_HIP + hipAmp * phA, kneeA);
        setLeg("rr", STAND_HIP + hipAmp * phA, kneeA);
        setLeg("fr", STAND_HIP + hipAmp * phB, kneeB);
        setLeg("rl", STAND_HIP + hipAmp * phB, kneeB);

        // Body dynamics
        // Double-frequency bounce (body rises at mid-swing of each pair)
        bodyBounce = 0.015 * S * Math.abs(Math.sin(walkPhase * 2));
    }

    // Named poses (validated by firmware/test/pose_generator)
    var POSES = {
        stand:    [0.3000, -0.6000, 0.3000, -0.6000, 0.3000, -0.6000, 0.3000, -0.6000],
        tall:     [0.1500, -0.5406, 0.1500, -0.5406, 0.1500, -0.5406, 0.1500, -0.5406],
        crouch:   [0.5000, -0.8000, 0.5000, -0.8000, 0.5000, -0.8000, 0.5000, -0.8000],
        rest:     [0.4000, -0.5629, 0.4000, -0.5629, 0.4000, -0.5629, 0.4000, -0.5629],
        alert:    [0.2000, -0.5691, 0.2000, -0.5691, 0.4000, -0.5629, 0.4000, -0.5629],
        sit:      [0.0500, -0.4639, 0.0500, -0.4639, 1.0000, -1.4000, 1.0000, -1.4000],
        lie_down: [0.8000, -1.2000, 0.8000, -1.2000, 0.8000, -1.2000, 0.8000, -1.2000],
        play_bow: [0.7000, -1.0000, 0.7000, -1.0000, 0.1500, -0.5406, 0.1500, -0.5406],
    };

    var currentPoseName = "stand";
    var activePose = null;   // angles currently held (persists after interpolation)
    var targetPose = null;   // angles being interpolated toward

    function setPose(name) {
        var angles = POSES[name];
        if (!angles) return;
        currentPoseName = name;
        targetPose = angles.slice();
        activePose = angles.slice();
        currentAction = null;
        currentMotion = "stop";
    }

    function clearPose() {
        activePose = null;
        targetPose = null;
        currentPoseName = "stand";
    }

    // Compute body height for a given pose by finding the lowest foot
    function poseBodyHeight(angles) {
        // Forward kinematics: foot Y relative to hip
        // foot_y = -upper*cos(hip) - lower*cos(hip+|knee|)
        var minFootY = 0;
        for (var i = 0; i < 4; i++) {
            var hip = angles[i * 2];
            var knee = angles[i * 2 + 1];
            var footY = -(UPPER_LEN * Math.cos(hip) + LOWER_LEN * Math.cos(hip + Math.abs(knee)));
            footY += HIP_OFFSET_Y;
            if (footY < minFootY) minFootY = footY;
        }
        return -minFootY + FOOT_R;
    }

    function animateAction() {
        // Hold active pose or interpolate to target
        var holdAngles = targetPose || activePose;
        if (holdAngles) {
            var legNames = ["fl", "fr", "rl", "rr"];
            for (var i = 0; i < 4; i++) {
                var leg = legs[legNames[i]];
                if (!leg) continue;
                var th = holdAngles[i * 2];
                var tk = holdAngles[i * 2 + 1];
                if (targetPose) {
                    // Interpolate
                    var ch = leg.hipPivot.rotation.z;
                    var ck = leg.kneePivot.rotation.z;
                    var dh = th - ch, dk = tk - ck;
                    if (Math.abs(dh) > 0.005 || Math.abs(dk) > 0.005) {
                        setLeg(legNames[i], ch + dh * 0.1, ck + dk * 0.1);
                    } else {
                        setLeg(legNames[i], th, tk);
                    }
                } else {
                    // Hold
                    setLeg(legNames[i], th, tk);
                }
            }

            // Check if interpolation is done
            if (targetPose) {
                var done = true;
                for (var j = 0; j < 4; j++) {
                    var l = legs[legNames[j]];
                    if (!l) continue;
                    if (Math.abs(l.hipPivot.rotation.z - targetPose[j*2]) > 0.005 ||
                        Math.abs(l.kneePivot.rotation.z - targetPose[j*2+1]) > 0.005) {
                        done = false;
                        break;
                    }
                }
                if (done) targetPose = null; // keep activePose for holding
            }

            // Adjust body height for the pose
            bodyBounce = poseBodyHeight(holdAngles) - standingHeight();
            return;
        }

        var a = currentAction;
        if (a === 1) {
            setLeg("fl", -0.8, -1.2);
            setLeg("fr", STAND_HIP, STAND_KNEE);
            setLeg("rl", STAND_HIP * 1.3, STAND_KNEE * 0.8);
            setLeg("rr", STAND_HIP * 1.3, STAND_KNEE * 0.8);
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

    // --- Ultrasonic beam + hit point ---
    var ultraHit = null; // hit point marker (added to scene, not dogGroup)

    function initUltraHit() {
        // Tall vertical beacon at hit point — sticks above walls
        var BEACON_HEIGHT = WALL_HEIGHT + 1.5;
        ultraHit = new THREE.Group();
        ultraHit.visible = false;
        scene.add(ultraHit);

        // Vertical line (thin cylinder from ground to above wall height)
        var poleGeo = new THREE.CylinderGeometry(0.008 * S, 0.008 * S, BEACON_HEIGHT, 6);
        var poleMat = new THREE.MeshBasicMaterial({ color: 0xff3333 });
        var pole = new THREE.Mesh(poleGeo, poleMat);
        pole.position.y = BEACON_HEIGHT / 2;
        ultraHit.add(pole);

        // Diamond marker at top
        var diamondGeo = new THREE.OctahedronGeometry(0.04 * S, 0);
        var diamondMat = new THREE.MeshBasicMaterial({ color: 0xff3333 });
        var diamond = new THREE.Mesh(diamondGeo, diamondMat);
        diamond.position.y = BEACON_HEIGHT;
        ultraHit.add(diamond);

        // Ground ring at base
        var ringGeo = new THREE.RingGeometry(0.04 * S, 0.06 * S, 16);
        var ringMat = new THREE.MeshBasicMaterial({
            color: 0xff3333, side: THREE.DoubleSide,
            transparent: true, opacity: 0.5,
        });
        var ring = new THREE.Mesh(ringGeo, ringMat);
        ring.rotation.x = -Math.PI / 2;
        ring.position.y = 0.02;
        ultraHit.add(ring);

        // Store refs for color updates
        ultraHit.userData.pole = pole;
        ultraHit.userData.diamond = diamond;
    }

    function updateUltraBeam() {
        if (ultraDistance == null || ultraDistance > 2500) {
            if (ultraBeam) ultraBeam.visible = false;
            if (ultraHit) ultraHit.visible = false;
            return;
        }
        if (!ultraBeam) return;
        ultraBeam.visible = true;

        // Convert mm to scene units, clamp
        var len = (ultraDistance / 1000) * S;
        var maxLen = 3.0 * S;
        len = Math.min(len, maxLen);

        // Cone spans from sensor to wall hit point
        // len = center-to-wall distance; sx = center-to-sensor offset
        var sx = ultraBeam.userData.sensorX;
        var sy = ultraBeam.userData.sensorY;
        var beamLen = Math.max(0.01, len - sx); // sensor-to-wall distance

        var lengthScale = beamLen / (1.0 * S);
        ultraBeam.scale.y = lengthScale;
        var widthScale = 0.5 + 0.5 * (beamLen / maxLen);
        ultraBeam.scale.x = widthScale;
        ultraBeam.scale.z = widthScale;

        // Cone center is halfway between sensor and wall
        ultraBeam.position.set(sx + (beamLen / 2), sy, 0);

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

        // Hit point: compute directly from target (non-interpolated) position + heading
        // Distance is from dog center (not sensor), so don't add sensor offset
        if (ultraHit) {
            var cosY = Math.cos(targetYaw);
            var sinY = Math.sin(targetYaw);
            var hitWorldX = targetX + len * cosY;
            var hitWorldZ = targetZ - len * sinY;
            ultraHit.position.set(hitWorldX, 0, hitWorldZ);
            ultraHit.visible = true;

            // Color beacon by distance
            var hitColor = ultraDistance < 150 ? 0xff2222 :
                           ultraDistance < 400 ? 0xff8800 : 0x33aaff;
            ultraHit.userData.pole.material.color.setHex(hitColor);
            ultraHit.userData.diamond.material.color.setHex(hitColor);
        }
    }

    // --- Camera ---

    var camTargetX = 0, camTargetZ = 0;

    function updateCameraPosition() {
        // Smoothly follow dog position
        camTargetX += (currentX - camTargetX) * 0.08;
        camTargetZ += (currentZ - camTargetZ) * 0.08;

        // Smoothly follow dog yaw (with angle wrapping)
        var yawDiff = currentYaw - camYaw;
        if (yawDiff > Math.PI) yawDiff -= 2 * Math.PI;
        if (yawDiff < -Math.PI) yawDiff += 2 * Math.PI;
        camYaw += yawDiff * 0.05;

        // Chase camera: orbit behind the dog
        // Dog faces +X at yaw=0. Camera at -X needs sin=-1,cos=0 → angle=3π/2
        var chaseAngle = camYaw + Math.PI * 1.5 + cameraAngle.thetaOffset;
        var r = cameraAngle.radius;
        var phi = cameraAngle.phi;

        // Look-at point: just forward and above the dog's nose
        var fwdOffset = BODY_L * 0.8; // forward from dog center
        var lookX = camTargetX + fwdOffset * Math.cos(camYaw);
        var lookZ = camTargetZ - fwdOffset * Math.sin(camYaw);
        var lookY = standingHeight() + BODY_H * 1.5;

        camera.position.x = lookX + r * Math.cos(phi) * Math.sin(chaseAngle);
        camera.position.y = lookY + r * Math.sin(phi);
        camera.position.z = lookZ + r * Math.cos(phi) * Math.cos(chaseAngle);
        camera.lookAt(lookX, lookY, lookZ);
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
            cameraAngle.thetaOffset -= dx * 0.008;
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

        // Smooth interpolation
        currentPitch += (targetPitch - currentPitch) * 0.15;
        currentRoll += (targetRoll - currentRoll) * 0.15;
        currentX += (targetX - currentX) * 0.15;
        currentZ += (targetZ - currentZ) * 0.15;
        // Smooth yaw with wrapping
        var yawDiff = targetYaw - currentYaw;
        if (yawDiff > Math.PI) yawDiff -= 2 * Math.PI;
        if (yawDiff < -Math.PI) yawDiff += 2 * Math.PI;
        currentYaw += yawDiff * 0.15;

        if (dogGroup) {
            // Position: sim X = scene X, sim Y = scene Z
            dogGroup.position.x = currentX;
            dogGroup.position.y = standingHeight() + bodyBounce;
            dogGroup.position.z = currentZ;

            // Yaw rotation around Y axis, then pitch/roll on top
            dogGroup.rotation.y = currentYaw;
            dogGroup.rotation.z = currentPitch * (Math.PI / 180);
            dogGroup.rotation.x = currentRoll * (Math.PI / 180);

            // Leg animation: use sim joints if available, otherwise gait from motion state
            // (gait updates bodyBounce and may add pitch oscillation)
            if (simJoints) {
                applySimJoints();
                bodyBounce = 0;
            } else {
                animateGait(dt);
            }

            updateUltraBeam();
        }

        updateOverlay();
        updateCameraPosition();
        renderer.render(scene, camera);
    }

    function onResize() {
        if (!container || !camera || !renderer) return;
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    }

    // --- Scan-point walls ---
    var wallMeshes = [];
    var wallTexture = null;
    var WALL_HEIGHT = 2.0; // scene units (~1.7x dog height)
    var WALL_THICKNESS = 0.08 * S;
    var CONNECT_DIST = 0.35 * S; // max gap between points to form a wall panel

    function createWallTexture() {
        // Procedural concrete panel texture — robotics test facility style
        var size = 256;
        var c = document.createElement("canvas");
        c.width = size; c.height = size;
        var ctx = c.getContext("2d");

        // Base concrete gray with slight warm tint
        ctx.fillStyle = "#8a8a86";
        ctx.fillRect(0, 0, size, size);

        // Noise grain
        var imgData = ctx.getImageData(0, 0, size, size);
        var d = imgData.data;
        for (var i = 0; i < d.length; i += 4) {
            var n = (Math.random() - 0.5) * 18;
            d[i] += n; d[i + 1] += n; d[i + 2] += n;
        }
        ctx.putImageData(imgData, 0, 0);

        // Horizontal panel seams (every 64px = ~0.3m panels)
        ctx.strokeStyle = "rgba(50, 50, 48, 0.6)";
        ctx.lineWidth = 2;
        for (var y = 64; y < size; y += 64) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(size, y);
            ctx.stroke();
            // Highlight line below seam
            ctx.strokeStyle = "rgba(160, 160, 155, 0.3)";
            ctx.beginPath();
            ctx.moveTo(0, y + 2);
            ctx.lineTo(size, y + 2);
            ctx.stroke();
            ctx.strokeStyle = "rgba(50, 50, 48, 0.6)";
        }

        // Vertical panel seams (every 128px = ~0.6m panels)
        for (var x = 128; x < size; x += 128) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, size);
            ctx.stroke();
            ctx.strokeStyle = "rgba(160, 160, 155, 0.3)";
            ctx.beginPath();
            ctx.moveTo(x + 2, 0);
            ctx.lineTo(x + 2, size);
            ctx.stroke();
            ctx.strokeStyle = "rgba(50, 50, 48, 0.6)";
        }

        // Subtle measurement markings — small ticks every 32px
        ctx.strokeStyle = "rgba(200, 180, 60, 0.2)";
        ctx.lineWidth = 1;
        for (var ty = 32; ty < size; ty += 32) {
            ctx.beginPath();
            ctx.moveTo(0, ty);
            ctx.lineTo(8, ty);
            ctx.stroke();
        }

        // Bottom safety stripe
        var stripeH = 12;
        ctx.fillStyle = "rgba(200, 160, 20, 0.15)";
        ctx.fillRect(0, size - stripeH, size, stripeH);

        var tex = new THREE.CanvasTexture(c);
        tex.wrapS = THREE.RepeatWrapping;
        tex.wrapT = THREE.RepeatWrapping;
        tex.repeat.set(1, 1);
        return tex;
    }

    function clearWalls() {
        wallMeshes.forEach(function (m) {
            if (m.geometry) m.geometry.dispose();
            scene.remove(m);
        });
        wallMeshes = [];
    }

    function buildWallsFromSegments(walls) {
        clearWalls();
        if (!walls || walls.length < 1) return;
        if (!wallTexture) wallTexture = createWallTexture();

        var wallMat = new THREE.MeshStandardMaterial({
            map: wallTexture,
            roughness: 0.85,
            metalness: 0.05,
            side: THREE.DoubleSide,
        });

        var edgeMat = new THREE.LineBasicMaterial({
            color: 0x555555, transparent: true, opacity: 0.4,
        });

        for (var i = 0; i < walls.length; i++) {
            var w = walls[i];
            // Convert sim coords to scene: sim (x,y) → scene (x*S, y*S as z)
            var sx1 = w.x1 * S, sz1 = w.y1 * S;
            var sx2 = w.x2 * S, sz2 = w.y2 * S;
            var dx = sx2 - sx1, dz = sz2 - sz1;
            var len = Math.sqrt(dx * dx + dz * dz);
            if (len < 0.01) continue;

            var cx = (sx1 + sx2) / 2;
            var cz = (sz1 + sz2) / 2;
            var angle = Math.atan2(dz, dx);
            var height = (w.height || 0.2) * S;

            var geo = new THREE.BoxGeometry(len, height, WALL_THICKNESS);
            var mesh = new THREE.Mesh(geo, wallMat);
            mesh.position.set(cx, height / 2, cz);
            mesh.rotation.y = -angle;
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            scene.add(mesh);
            wallMeshes.push(mesh);

            var edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), edgeMat);
            edges.position.copy(mesh.position);
            edges.rotation.copy(mesh.rotation);
            scene.add(edges);
            wallMeshes.push(edges);
        }
    }

    // --- Debug overlay: wireframe body, joint indicators, IMU axes ---
    var overlayVisible = false;
    var overlayParts = [];  // all overlay objects to toggle visibility
    var jointMarkers = {};

    function buildOverlay() {
        if (overlayParts.length > 0) return;

        function addOverlay(obj, parent) {
            obj.visible = false;
            obj.traverse(function (c) { c.userData.isOverlay = true; });
            (parent || dogGroup).add(obj);
            overlayParts.push(obj);
        }

        // Wireframe body box
        var bodyGeo = new THREE.BoxGeometry(BODY_L * 1.05, BODY_H * 1.3, BODY_W * 1.05);
        var wireframe = new THREE.LineSegments(
            new THREE.EdgesGeometry(bodyGeo),
            new THREE.LineBasicMaterial({ color: 0x00ffff })
        );
        addOverlay(wireframe);

        // IMU coordinate axes (RGB = XYZ)
        var axisLen = BODY_L * 1.2;
        var axes = new THREE.Group();
        axes.add(makeArrow(0, 0, 0, axisLen, 0, 0, 0xff4444));
        axes.add(makeArrow(0, 0, 0, 0, axisLen * 0.7, 0, 0x44ff44));
        axes.add(makeArrow(0, 0, 0, 0, 0, axisLen * 0.7, 0x4488ff));
        addOverlay(axes);

        // Horizon ring — in scene space (not dogGroup)
        var ringGeo = new THREE.RingGeometry(BODY_L * 0.7, BODY_L * 0.75, 32);
        var ring = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({
            color: 0x00ffcc, transparent: true, opacity: 0.35, side: THREE.DoubleSide
        }));
        ring.rotation.x = -Math.PI / 2;
        ring.visible = false;
        ring.userData.isOverlay = true;
        scene.add(ring);
        overlayParts.push(ring);
        // Store ref for position update
        ring.userData.isHorizon = true;

        // Per-leg: joint markers + wireframe links
        ["fl", "fr", "rl", "rr"].forEach(function (name) {
            var leg = legs[name];
            if (!leg) return;

            var hipMarker = makeGlowMarker(0, 0, 0, 0x00ffcc, 0.012 * S);
            addOverlay(hipMarker, leg.hipPivot);

            var kneeMarker = makeGlowMarker(0, 0, 0, 0xff8800, 0.012 * S);
            addOverlay(kneeMarker, leg.kneePivot);

            var footMarker = makeGlowMarker(0, -LOWER_LEN, 0, 0xff44ff, 0.008 * S);
            addOverlay(footMarker, leg.kneePivot);

            addOverlay(makeLegLink(0, 0, 0, 0, -UPPER_LEN, 0, 0x00ffff), leg.hipPivot);
            addOverlay(makeLegLink(0, 0, 0, 0, -LOWER_LEN, 0, 0x00cccc), leg.kneePivot);

            jointMarkers[name] = { hip: hipMarker, knee: kneeMarker, foot: footMarker };
        });
    }

    function makeArrow(x1, y1, z1, x2, y2, z2, color) {
        var group = new THREE.Group();
        // Shaft — cylinder
        var dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
        var len = Math.sqrt(dx * dx + dy * dy + dz * dz);
        var shaft = new THREE.Mesh(
            new THREE.CylinderGeometry(0.008 * S, 0.008 * S, len, 6),
            new THREE.MeshBasicMaterial({ color: color })
        );
        shaft.position.set((x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2);
        shaft.lookAt(x2, y2, z2);
        shaft.rotateX(Math.PI / 2);
        group.add(shaft);
        // Tip sphere
        var tip = new THREE.Mesh(
            new THREE.SphereGeometry(0.02 * S, 8, 8),
            new THREE.MeshBasicMaterial({ color: color })
        );
        tip.position.set(x2, y2, z2);
        group.add(tip);
        return group;
    }

    function makeLegLink(x1, y1, z1, x2, y2, z2, color) {
        var geo = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(x1, y1, z1),
            new THREE.Vector3(x2, y2, z2)
        ]);
        var line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: color }));
        line.userData.isOverlay = true;
        return line;
    }

    function makeGlowMarker(x, y, z, color, radius) {
        var group = new THREE.Group();
        // Core sphere
        var core = new THREE.Mesh(
            new THREE.SphereGeometry(radius, 10, 10),
            new THREE.MeshBasicMaterial({ color: color })
        );
        group.add(core);
        // Outer glow ring
        var ring = new THREE.Mesh(
            new THREE.RingGeometry(radius * 1.3, radius * 2.0, 16),
            new THREE.MeshBasicMaterial({
                color: color, transparent: true, opacity: 0.3,
                side: THREE.DoubleSide, depthWrite: false
            })
        );
        group.add(ring);
        group.position.set(x, y, z);
        return group;
    }

    function updateOverlay() {
        if (!overlayVisible) return;

        // Horizon ring: stays level at dog's world position
        overlayParts.forEach(function (obj) {
            if (obj.userData.isHorizon && dogGroup) {
                obj.position.set(
                    dogGroup.position.x,
                    dogGroup.position.y,
                    dogGroup.position.z
                );
            }
        });
    }

    function toggleOverlay(show) {
        if (overlayParts.length === 0) buildOverlay();
        overlayVisible = show;

        // Show/hide overlay parts
        overlayParts.forEach(function (obj) { obj.visible = show; });

        // Show/hide normal dog meshes (inverse of overlay)
        if (dogGroup) {
            dogGroup.traverse(function (child) {
                if (child.isMesh && !child.userData.isOverlay) {
                    child.visible = !show;
                }
                if (child.isLine && !child.userData.isOverlay) {
                    child.visible = !show;
                }
            });
        }
    }

    // --- Public API ---

    return {
        init: init,

        // Debug: expose current state for testing
        debug: function () {
            return {
                targetX: targetX, targetZ: targetZ, targetYaw: targetYaw,
                currentX: currentX, currentZ: currentZ, currentYaw: currentYaw,
                headingDeg: targetYaw * (180 / Math.PI),
                motion: currentMotion,
                ultraDistance: ultraDistance,
            };
        },

        updateIMU: function (msg) {
            targetPitch = msg.pitch;
            targetRoll = msg.roll;
            if (msg.joints) simJoints = msg.joints;
        },

        updateOdometry: function (msg) {
            if (msg.motion != null) {
                currentMotion = msg.motion;
                // Movement clears any held pose
                if (msg.motion !== "stop") clearPose();
            }
            if (msg.x != null && msg.y != null) {
                targetX = msg.x * S;
                targetZ = msg.y * S;
            }
            // rotation.y = +90° faces -Z, but heading=90° (turned left) should
            // face +Z (sim +Y), so negate to get correct visual direction
            if (msg.heading != null) {
                targetYaw = -msg.heading * (Math.PI / 180);
            }
        },

        updateUltrasonic: function (distance_mm) {
            ultraDistance = distance_mm;
        },

        reset: function () {
            targetPitch = targetRoll = 0;
            currentPitch = currentRoll = 0;
            targetX = targetZ = targetYaw = 0;
            currentX = currentZ = currentYaw = 0;
            camTargetX = camTargetZ = 0;
            currentMotion = "stop";
            currentAction = null;
            walkPhase = 0;
            simJoints = null;
            clearWalls();
            if (dogGroup) {
                dogGroup.position.set(0, standingHeight(), 0);
                dogGroup.rotation.set(0, 0, 0);
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
            return Object.keys(POSES);
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
