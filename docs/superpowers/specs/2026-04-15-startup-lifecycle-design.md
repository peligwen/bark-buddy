# Startup & Lifecycle State Machine Design

## Context

The MechDog currently stands up immediately on firmware boot and stays standing indefinitely in `STAND` state. There's no concept of an operator being "in control" at the firmware level — the heartbeat watchdog (5s) is the only fallback, and it only fires when the host connection drops entirely.

The desired behavior: the dog should boot, briefly stand to show it's alive, then lay back down if nobody takes control within 10 seconds. When an operator acquires the control lock, the dog wakes up and stands ready. When the lock is released, it counts down and lays back down. The physical rest pose should look like a dog resting — hips back, knees tucked — not the current geometric-center 1500µs neutral.

## Lifecycle State Machine

### States

| State | Servos | Description |
|-------|--------|-------------|
| `BOOTING` | Off | Hardware init: IMU, sonar, offsets, LEDs. Transient (~1s). |
| `WAKING` | Ramping up | Smooth ramp from rest pose → standing pose over ~2s. |
| `IDLE` | Holding stand | Standing, no active operator. 10s countdown to sleep. |
| `ACTIVE` | Full control | Operator has control. Gait engine responds to commands. |
| `SLEEPING` | Ramping down | Smooth ramp from current pose → rest pose over ~1.5s. |
| `RESTING` | Detached | Servos off. Minimal power. Waiting for wake command. |

### Transitions

```
BOOTING ──(init complete)──→ WAKING ──(ramp done)──→ IDLE
                                                       │
                              ┌── cmd_wake ────────────┤
                              ↓                        │
                           ACTIVE                      │
                              │                        │
                              ├── cmd_sleep ──→ IDLE ──┤
                              │                        │
                              └── heartbeat timeout    │
                                  (safety) ──→ SLEEPING│
                                                       │
                                     10s timeout ──→ SLEEPING ──(ramp done)──→ RESTING
                                                                                 │
                                                       cmd_wake ────────→ WAKING ┘
```

### Transition Details

| From | Trigger | To | Action |
|------|---------|-----|--------|
| `BOOTING` | Init complete | `WAKING` | Attach servos at rest pose, begin ramp |
| `WAKING` | Ramp complete | `IDLE` | Start 10s countdown |
| `WAKING` | `cmd_wake` received | `ACTIVE` | Continue ramp, but enter ACTIVE when done (skip IDLE) |
| `IDLE` | `cmd_wake` | `ACTIVE` | Cancel countdown |
| `IDLE` | 10s timeout | `SLEEPING` | Begin ramp to rest pose |
| `ACTIVE` | `cmd_sleep` | `IDLE` | Start 10s countdown (grace period before full sleep) |
| `ACTIVE` | Heartbeat timeout (5s) | `SLEEPING` | Safety fallback — host vanished |
| `SLEEPING` | Ramp complete | `RESTING` | Detach all servos |
| `SLEEPING` | `cmd_wake` received | `WAKING` | Reverse direction, ramp back to standing |
| `RESTING` | `cmd_wake` | `WAKING` | Re-attach servos, begin ramp |
| Any | Low battery | `RESTING` | Immediate detach (no ramp) — safety override |

### Edge Cases

- **`cmd_wake` during SLEEPING:** The dog is mid-ramp going down. Capture current servo positions as new start point, reverse ramp toward standing. No snap.
- **`cmd_sleep` during WAKING:** Capture current positions, reverse ramp toward rest. Enters SLEEPING.
- **`cmd_wake` during IDLE:** Simply cancel the countdown, enter ACTIVE. No servo movement needed — already standing.
- **Rapid lock/unlock toggling:** Each transition captures current servo positions for smooth interpolation. No state can be skipped — the ramps always pass through the current physical position.
- **`cmd_move` in non-ACTIVE state:** Ignored. Firmware only processes motion commands in ACTIVE. ACK with `ok: false, reason: "not_active"`.
- **Heartbeat timeout in IDLE:** Treated same as the 10s idle timeout — transitions to SLEEPING.

## Protocol Changes

### New Commands (host → firmware)

```json
{"type": "cmd_wake"}
{"type": "cmd_sleep"}
```

### Updated Telemetry

The existing `telem_status` message gains a `lifecycle` field:

```json
{"type": "telem_status", "mode": "idle", "lifecycle": "active", "balance": true, "servos": true}
```

`lifecycle` values: `"booting"`, `"waking"`, `"idle"`, `"sleeping"`, `"resting"`, `"active"`

### ACK Changes

Motion commands (`cmd_move`, `cmd_gait_params`, etc.) received outside of `ACTIVE` state return:

```json
{"type": "ack", "ref_type": "cmd_move", "ok": false, "reason": "not_active"}
```

## Rest Pose

The current `LYING_DOWN_POSE` (all 1500µs) is a geometric center, not a natural rest. We need to tune a real "hips back, knees tucked" rest pose.

**Approach:**
1. Use `cmd_servo` in test mode to manually find natural rest pulse widths for all 8 servos
2. Target: hips rotated back (~45° from standing), knees tucked in tight
3. Store as `REST_POSE[8]` in `config.h`, replacing `LYING_DOWN_POSE`
4. Corresponding sim angles: `REST_HIP` and `REST_KNEE` in `physics.py`

**Constraints:**
- All transitions ramp through this pose (boot-up starts here, shutdown ends here)
- Must be physically stable — the dog should be able to rest at this pose with servos detached without toppling
- Frail mode limits still apply during ramp (slew rate, max offset from target)

## Firmware Implementation

### Files Modified

- **`firmware/include/config.h`**: Add `REST_POSE[8]`, `IDLE_TIMEOUT_MS = 10000`, rename/replace `LYING_DOWN_POSE`
- **`firmware/include/gait.h`**: Add lifecycle states enum (`LifecycleState`), declare lifecycle functions
- **`firmware/src/gait.cpp`**: Implement lifecycle state machine, ramp logic, idle countdown
- **`firmware/src/main.cpp`**: Change `setup()` to use new lifecycle boot sequence; route `cmd_wake`/`cmd_sleep` through command handler; update heartbeat timeout to trigger lifecycle transition instead of direct servo control
- **`firmware/src/command_handlers.cpp`**: Add handlers for `cmd_wake`, `cmd_sleep`
- **`firmware/src/servos.cpp`**: Refactor `servos_init()` — separate "attach at pose" from "ramp to standing". Current `servos_shutdown_to_lying_down()` becomes a general `servos_ramp_to(target[], duration_ms, steps)` utility.

### Key Refactoring

`servos.cpp` currently has hardcoded ramp logic in both `servos_init()` (lie→stand) and `servos_shutdown_to_lying_down()` (current→lie). These should be unified into a single `servos_ramp_to(target_us[], duration_ms)` function that the lifecycle state machine calls with appropriate targets and timing.

### Existing Code to Reuse

- `servos_write_us()` — already writes arbitrary pulse widths to servos
- `servos_detach_all()` — used for RESTING state
- Ramp interpolation math from `servos_init()` lines 56-67 — extract into `servos_ramp_to()`
- `gait_set_state()` — lifecycle can set gait to STAND when entering ACTIVE, STOP when leaving
- Command handler dispatch table in `command_handlers.cpp` — add `cmd_wake`/`cmd_sleep` entries

## Host Implementation

### Files Modified

- **`host/server.py`**: On lock acquired → `cmd_wake`; on lock released/timeout → `cmd_sleep`. Broadcast lifecycle state to browser via WebSocket. Remove the old REPL-based idle timeout (transform lowering hack).
- **`host/comms.py`**: Add `wake()` and `sleep()` methods that send the new JSON commands.
- **`host/firmware_transport.py`**: Parse new `lifecycle` field from `telem_status`.
- **`host/sim/sim_transport.py`**: Implement lifecycle state machine mirroring firmware. Handle `cmd_wake`/`cmd_sleep`.
- **`host/sim/physics.py`**: Add `REST_HIP`, `REST_KNEE` constants. Add rest pose support and ramp transitions.

### Lock → Lifecycle Mapping

```python
# In server.py, when lock state changes:

async def _on_lock_acquired(self):
    await self._dog.wake()

async def _on_lock_released(self):
    await self._dog.sleep()
```

The host's `_lock_timeout` (30s auto-release) already handles the case of an abandoned browser tab — when the lock auto-releases, `cmd_sleep` fires, and the firmware's 10s IDLE countdown begins.

## Web UI Changes

- Display lifecycle state somewhere in the telemetry overlay (small status indicator)
- D-pad / motion controls should be visually disabled when lifecycle is not `active`
- When user tries to send commands while not active, show a message like "Acquire control to command the dog"

## Sim Parity

The sim mirrors the firmware lifecycle exactly:
- `SimTransport` maintains the same state machine
- `DogPhysics` handles rest pose angles and ramp transitions
- The 10s idle timeout runs in the sim's event loop
- The sim's "heartbeat" equivalent: if the host process crashes, the sim just stops (no separate safety concern)

## Verification

1. **Firmware unit tests**: Test lifecycle transitions (all valid transitions, invalid command rejection, edge cases like wake-during-sleep)
2. **Boot sequence test**: Flash firmware, observe: rest pose → stand → 10s → lay down → detach
3. **Lock cycle test**: Acquire lock in browser → dog stands → release lock → 10s → dog lays down → re-acquire → dog stands
4. **Heartbeat safety test**: Acquire lock → kill host process → 5s → dog lays down
5. **Sim parity test**: Run in sim mode, verify same lifecycle transitions in 3D view
6. **Rapid toggle test**: Rapidly acquire/release lock, verify smooth servo transitions with no snapping
7. **Battery safety test**: Verify low battery still immediately detaches regardless of lifecycle state
