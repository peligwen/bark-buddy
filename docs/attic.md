# Attic

All items below were removed from `main` during Phase 1 of the kernel reshape (2026-04-19).
The `pre-reshape` branch preserves every file at its pre-cut state.

## Mapping / navigation behaviors
- `host/behaviors/scan.py` — sonar scan behavior; mapping rewrite pending SLAM
- `host/behaviors/map_store.py` — scan point store; mapping rewrite pending SLAM
- `host/behaviors/wall_fit.py` — RANSAC wall fitting; mapping rewrite pending SLAM
- `host/behaviors/wall_mesh.py` — wall mesh builder; mapping rewrite pending SLAM
- `host/behaviors/octree.py` — spatial index for scan points; mapping rewrite pending SLAM

## Calibration / sweep tooling
- `host/sweep/__init__.py` — sweep package init; calibration tooling, not kernel
- `host/sweep/__main__.py` — sweep entry point; calibration tooling, not kernel
- `host/sweep/runner.py` — sweep orchestrator; calibration tooling, not kernel
- `host/sweep/worker.py` — sweep worker; calibration tooling, not kernel
- `host/sweep/sampler.py` — pose sampler; calibration tooling, not kernel
- `host/sweep/scenarios.py` — sweep scenarios; calibration tooling, not kernel
- `host/sweep/results.py` — sweep result analysis; calibration tooling, not kernel
- `host/sweep/scoring.py` — sweep scoring; calibration tooling, not kernel
- `host/calibrate_servos.py` — interactive servo calibration; not kernel, one-time setup
- `host/capture_pose.py` — servo pose capture tool; not kernel, one-time setup
- `host/identify_servos.py` — servo identification sweep; not kernel, one-time setup
- `host/apply_to_firmware.py` — writes calibration to firmware; not kernel, one-time setup
- `host/servo_test.py` — servo range test; not kernel, one-time setup
- `host/map_servos.py` — servo mapping tool; not kernel, one-time setup (was untracked, removed)
- `host/probe_stock_firmware.py` — probes vendor firmware; not kernel, superseded by custom firmware (was untracked, removed)

## Ad-hoc test scripts
- `host/test_server.py` — manual server test; non-pytest, used removed cmd_action
- `host/test_mapping.py` — manual mapping test; ad-hoc, replaced by firmware unit tests
- `host/test_scan.py` — manual scan test; ad-hoc, replaced by firmware unit tests
- `host/test_wall_mesh.py` — manual wall mesh test; ad-hoc, replaced by firmware unit tests
- `host/test_button_engage.py` — manual button test; ad-hoc, not part of test suite

## Secondary web UIs
- `web/tuning.html` — offset/servo-pin tuning UI; UX will be folded into primary UI in Phase 3
- `web/gait.html` — standalone gait parameter UI; duplicate UI, covered by primary UI controls
- `web/modules/map.js` — 2D scan map renderer; companion to deleted scan/map behavior
- `web/dog3d/sonar.js` — sonar beam visualization; scan-map feature, not in kernel 3D view
- `web/dog3d/walls.js` — wall mesh visualization; scan-map feature, not in kernel 3D view

## Stale docs
- `docs/architecture.md` — superseded by new docs/architecture.md written in Phase 2
- `docs/protocol.md` — superseded by new docs/protocol.md written in Phase 2
- `docs/hardware-schematic.md` — absorbed into new docs/hardware.md in Phase 2
- `docs/decisions.md` — ad-hoc decision log; architecture decisions now in core docs
- `docs/implementation-plan.md` — early milestone plan; superseded by superpowers plans
- `docs/stock-firmware-analysis.md` — stock firmware analysis; custom-firmware-only path since 2026-04

## Completed plans and specs (ephemeral by design)
- `docs/superpowers/plans/2026-04-13-firmware-foundation.md` — FreeRTOS refactor; complete
- `docs/superpowers/plans/2026-04-13-ik-balance-transform-gait.md` — IK pipeline plan; complete
- `docs/superpowers/plans/2026-04-15-bark-cli-ui-trim.md` — CLI/UI trim; complete
- `docs/superpowers/plans/2026-04-15-remove-dead-transport-code.md` — transport cleanup; complete
- `docs/superpowers/plans/2026-04-15-startup-lifecycle.md` — lifecycle FSM plan; superseded by engage-switch refactor
- `docs/superpowers/plans/2026-04-16-custom-firmware-only-refactor.md` — custom-firmware-only refactor; complete
- `docs/superpowers/specs/2026-04-13-firmware-foundation-design.md` — firmware foundation design; complete
- `docs/superpowers/specs/2026-04-15-auto-connect-detect-update-design.md` — auto-connect design; complete
- `docs/superpowers/specs/2026-04-15-startup-lifecycle-design.md` — lifecycle design; superseded
- `docs/superpowers/specs/2026-04-15-web-ui-simplification-design.md` — web UI simplification design; complete
