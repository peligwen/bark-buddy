"""
Servo identification diagnostic for MechDog.

Drives a fixed sequence of single-servo dithers and pure-axis body-transform
sweeps, captures physical observations from the operator, and emits an A/B verdict:

  Scenario A — ik.h's non-standard leg-index scheme matches the physical wiring.
               "FR" and "RL" labels do NOT mean Front-Right / Rear-Left — they are
               side-pair shorthand from the Hiwonder stock firmware convention.
               The transform engine is correct; docs/physics.cpp have wrong assumptions.

  Scenario B — Standard naming (FR=front-right, RL=rear-left) matches the physical
               wiring, but ik.h maps leg 1 to physical rear-left and leg 2 to
               physical front-right. The transform engine produces incorrect motion
               on half the legs under pitch/roll/yaw.

Saves observations and verdict to host/servo_mapping.json on completion.

Usage:
    python scripts/servo_identify.py [--port /dev/cu.usbserial-XXXX]
    python scripts/servo_identify.py --tcp 192.168.1.x

Before running:
    Put a small tape tag on each leg: "FL" (front-left), "FR" (front-right),
    "RL" (rear-left), "RR" (rear-right). The front of the dog is the direction
    it walks forward (camera side if present). Stand the robot on a table.
"""

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "host"))

from dog.discover import find_serial_port
from dog.dog import Dog

# Mirrors firmware/include/config.h STANDING_POSE (indices 0..7)
STANDING_POSE_US = [2096, 1621, 904, 1379, 2170, 1611, 830, 1389]

# Code-assigned names (what config.h and hardware.md call them)
CODE_NAMES = [
    "FL_hip",  "FL_knee",
    "FR_hip",  "FR_knee",
    "RL_hip",  "RL_knee",
    "RR_hip",  "RR_knee",
]

EXCURSION_US = 50       # ±50 µs dither for Step 0
DITHER_PAUSE_S = 0.5    # pause between dither steps
INTER_SERVO_S = 1.0     # pause between servos
TRANSFORM_SETTLE_S = 1.2
TRANSFORM_RETURN_S = 1.0
ENGAGE_TIMEOUT_S = 4.0
MIN_BATTERY_MV = 6500

TRANSFORM_STEPS = [
    {
        "name": "dz",
        "label": "Step 1 — Body height (dz ±8 mm)",
        "desc": (
            "Expected: all four KNEES move the same direction; hips near-zero.\n"
            "This confirms hip vs knee identification within each leg."
        ),
        "fwd": {"z": 8, "ms": 1000},
        "rev": {"z": -8, "ms": 1000},
    },
    {
        "name": "dx",
        "label": "Step 2 — Body forward/back (dx ±8 mm)",
        "desc": (
            "Expected: all four HIPS move the same direction; knees near-zero.\n"
            "Confirms hip vs knee and that all four legs are active."
        ),
        "fwd": {"x": 8, "ms": 1000},
        "rev": {"x": -8, "ms": 1000},
    },
    {
        "name": "pitch",
        "label": "Step 3 — Body pitch (pitch ±5°)  ← KEY TEST",
        "desc": (
            "Expected: two legs extend (knees straighter), two legs compress (knees more bent).\n"
            "This is the A/B discriminator — which legs extend tells us the wiring convention."
        ),
        "fwd": {"pitch": 5, "ms": 1000},
        "rev": {"pitch": -5, "ms": 1000},
    },
    {
        "name": "roll",
        "label": "Step 4 — Body roll (roll ±5°)",
        "desc": (
            "Expected: two legs on one side compress, two on the other side extend.\n"
            "Cross-check for the orthogonal axis from Step 3."
        ),
        "fwd": {"roll": 5, "ms": 1000},
        "rev": {"roll": -5, "ms": 1000},
    },
    {
        "name": "yaw",
        "label": "Step 5 — Body yaw (yaw ±15°)",
        "desc": (
            "Expected: diagonal pattern — FL+RR hips move one way, FR+RL hips the other.\n"
            "(Using the physical legs you tagged, not the code names.)"
        ),
        "fwd": {"yaw": 15, "ms": 1000},
        "rev": {"yaw": -15, "ms": 1000},
    },
]


def _ts() -> str:
    return f"{time.monotonic():.2f}"


def prompt(msg: str) -> str:
    print(f"\n{msg}")
    print("  > ", end="", flush=True)
    return input().strip()


def prompt_leg(msg: str) -> str:
    """Prompt for a physical leg name, accepting FL/FR/RL/RR/none."""
    valid = {"FL", "FR", "RL", "RR", "none", "?"}
    while True:
        ans = prompt(f"{msg} [FL / FR / RL / RR / none / ?]").upper()
        if ans in valid:
            return ans
        print("  Please enter FL, FR, RL, RR, none, or ? (to skip with explanation).")


def prompt_joint(msg: str) -> str:
    """Prompt for hip or knee."""
    valid = {"hip", "knee", "both", "none", "?"}
    while True:
        ans = prompt(f"{msg} [hip / knee / both / none / ?]").lower()
        if ans in valid:
            return ans
        print("  Please enter hip, knee, both, none, or ?.")


def prompt_leg_set(msg: str) -> list[str]:
    """Prompt for a space-separated set of physical leg names."""
    valid = {"FL", "FR", "RL", "RR"}
    print(f"\n{msg}")
    print("  Enter space-separated legs from FL FR RL RR (e.g. 'FL FR') or 'skip':")
    print("  > ", end="", flush=True)
    raw = input().strip().upper()
    if raw in ("SKIP", ""):
        return []
    return [t for t in raw.split() if t in valid]


async def setup(dog: Dog) -> bool:
    """Connect, wait for boot, check battery, engage. Returns True if ready."""
    boot_event = asyncio.Event()

    def on_telem(msg: dict) -> None:
        if msg.get("type") == "boot":
            boot_event.set()

    dog.set_telem_callback(on_telem)
    await dog.open()

    print("[identify] waiting for boot frame (3 s)...")
    try:
        await asyncio.wait_for(boot_event.wait(), timeout=3.0)
        print(f"[identify] firmware: {dog.firmware_info}")
    except asyncio.TimeoutError:
        print("[identify] no boot frame — issuing ping, continuing")
        await dog.send_json({"type": "ping"})
        await asyncio.sleep(0.5)

    await asyncio.sleep(1.0)

    batt = dog.get_battery_mv()
    print(f"[identify] battery: {batt} mV  cutoff: {dog.get_battery_cutoff()}")

    if dog.get_battery_cutoff():
        print("[identify] ABORT: battery cutoff latched — reboot robot to clear")
        return False
    if batt < MIN_BATTERY_MV:
        print(f"[identify] ABORT: battery {batt} mV below {MIN_BATTERY_MV} mV threshold")
        return False

    print("[identify] engaging...")
    await dog.send_json({"type": "cmd_engage", "enabled": True})
    ack = await dog.recv_ack("cmd_engage", timeout=ENGAGE_TIMEOUT_S)
    if ack is None or not ack.get("ok"):
        err = ack.get("error") if ack else "timeout"
        print(f"[identify] ABORT: engage failed — {err}")
        return False

    print("[identify] waiting for engage ramp...")
    deadline = asyncio.get_event_loop().time() + ENGAGE_TIMEOUT_S
    while asyncio.get_event_loop().time() < deadline:
        if dog.get_engaged() and not dog.get_ramping():
            break
        await asyncio.sleep(0.1)
    else:
        print("[identify] ABORT: never reached engaged state")
        return False

    print("[identify] ready\n")
    return True


async def teardown(dog: Dog) -> None:
    print("\n[identify] disengaging...")
    await dog.send_json({"type": "cmd_engage", "enabled": False})
    ack = await dog.recv_ack("cmd_engage", timeout=3.0)
    if ack is None:
        print("[identify] WARNING: disengage ack timeout")
    await asyncio.sleep(1.6)
    await dog.close()


async def run_step0(dog: Dog) -> dict:
    """Dither each servo individually and record physical observations."""
    print("=" * 60)
    print("STEP 0 — Per-index ground truth")
    print("=" * 60)
    print(
        "For each servo index, the robot will dither ±50 µs around standing.\n"
        "Watch WHICH PHYSICAL JOINT twitches, and report using your tape tags:\n"
        "  FL = front-left   FR = front-right\n"
        "  RL = rear-left    RR = rear-right\n"
        "  hip = the upper joint (thigh rotation)\n"
        "  knee = the lower joint (shin bend)\n"
    )
    _ = prompt("Press ENTER when ready (robot should be standing still)")

    observations = {}

    for idx in range(8):
        center = STANDING_POSE_US[idx]
        print(f"\n--- index {idx}  ({CODE_NAMES[idx]})  center={center} µs ---")

        for pulse in [center, center + EXCURSION_US, center, center - EXCURSION_US, center]:
            await dog.send_json({"type": "cmd_servo", "index": idx, "pulse_us": pulse})
            ack = await dog.recv_ack("cmd_servo", timeout=2.0)
            if ack is None:
                print(f"  TIMEOUT at {pulse} µs — skipping index {idx}")
                break
            if not ack.get("ok"):
                print(f"  NACK: {ack.get('error')} at {pulse} µs — skipping")
                break
            await asyncio.sleep(DITHER_PAUSE_S)

        leg = prompt_leg(f"  Which PHYSICAL LEG twitched for index {idx}?")
        joint = "?" if leg in ("none", "?") else prompt_joint(f"  Which JOINT on that leg?")
        note = ""
        if leg == "?" or joint == "?":
            note = prompt("  Brief note (what was ambiguous?)")

        observations[idx] = {"code_name": CODE_NAMES[idx], "leg": leg, "joint": joint}
        if note:
            observations[idx]["note"] = note

        print(f"  Recorded: index {idx} → {leg} {joint}")
        await asyncio.sleep(INTER_SERVO_S)

    return observations


async def run_transform_steps(dog: Dog) -> dict:
    """Run pure-axis transform sweeps and record observations."""
    print("\n" + "=" * 60)
    print("STEPS 1-5 — Body-transform sweeps")
    print("=" * 60)
    print(
        "The robot will sweep each transform axis forward and back.\n"
        "Observe which PHYSICAL legs (by your tape tags) moved and how.\n"
        "Return to neutral between steps — the script handles this automatically.\n"
    )
    _ = prompt("Press ENTER when ready")

    neutral = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0, "ms": 800}
    results = {}

    for step in TRANSFORM_STEPS:
        print(f"\n{'─' * 60}")
        print(step["label"])
        print(step["desc"])
        _ = prompt("Press ENTER to run this step")

        # Forward
        print(f"  → sending {step['fwd']}")
        await dog.send_json({"type": "cmd_transform", **step["fwd"]})
        await dog.recv_ack("cmd_transform", timeout=2.0)
        await asyncio.sleep(TRANSFORM_SETTLE_S)

        # Reverse
        print(f"  → sending {step['rev']}")
        await dog.send_json({"type": "cmd_transform", **step["rev"]})
        await dog.recv_ack("cmd_transform", timeout=2.0)
        await asyncio.sleep(TRANSFORM_SETTLE_S)

        # Return to neutral
        print("  → returning to neutral")
        await dog.send_json({"type": "cmd_transform", **neutral})
        await dog.recv_ack("cmd_transform", timeout=2.0)
        await asyncio.sleep(TRANSFORM_RETURN_S)

        # Collect observation
        print()
        if step["name"] == "pitch":
            extended = prompt_leg_set(
                "Which physical legs EXTENDED (knee straighter, body lower on that side)?"
            )
            compressed = prompt_leg_set(
                "Which physical legs COMPRESSED (knee more bent, body higher on that side)?"
            )
            results[step["name"]] = {"extended": extended, "compressed": compressed}
        elif step["name"] == "roll":
            extended = prompt_leg_set("Which physical legs EXTENDED under roll?")
            compressed = prompt_leg_set("Which physical legs COMPRESSED under roll?")
            results[step["name"]] = {"extended": extended, "compressed": compressed}
        elif step["name"] == "yaw":
            diag1 = prompt_leg_set("Which two physical legs moved same direction under yaw (group 1)?")
            diag2 = prompt_leg_set("Which two physical legs moved the OTHER direction (group 2)?")
            results[step["name"]] = {"group1": diag1, "group2": diag2}
        else:
            # dz and dx: just note anomalies
            anomaly = prompt(
                "Were all four legs behaving as expected (all same direction)?\n"
                "  Enter 'ok' or describe any anomaly"
            )
            results[step["name"]] = {"ok": anomaly.lower() == "ok", "note": anomaly}

    return results


def compute_verdict(step0: dict, transforms: dict) -> dict:
    """
    Classify as Scenario A or B using Step 0 index→physical mapping and
    the pitch observation (the key discriminator).

    Scenario A: ik.h treats legs 0+2 (indices 0,1,4,5) as "front" (hip_x=+85).
                Under pitch+12, physical front legs extend → physical front ↔ code indices {0,1,4,5}.

    Scenario B: Standard naming. Front legs are code indices {0,1,2,3} (legs 0 and 1 = FL+FR).
                Under pitch+12, physical front legs extend → physical front ↔ code indices {0,1,2,3}.
    """
    verdict = {
        "scenario": "UNDETERMINED",
        "confidence": "low",
        "anomalies": [],
        "pitch_analysis": {},
        "step0_summary": {},
    }

    # Build physical-leg → index set mapping from Step 0
    leg_to_indices: dict[str, list] = {"FL": [], "FR": [], "RL": [], "RR": []}
    dead_indices = []
    for idx_str, obs in step0.items():
        idx = int(idx_str)
        leg = obs.get("leg", "?")
        if leg in leg_to_indices:
            leg_to_indices[leg].append(idx)
        elif leg == "none":
            dead_indices.append(idx)

    verdict["step0_summary"] = {
        "leg_to_indices": {k: sorted(v) for k, v in leg_to_indices.items()},
        "dead_or_unknown": sorted(dead_indices),
    }

    if dead_indices:
        verdict["anomalies"].append(
            f"Dead/unresponsive indices: {dead_indices} — check wiring and SERVO_PINS in servos.cpp:13"
        )

    # Check hip/knee parity: even indices should be hips, odd should be knees
    for idx_str, obs in step0.items():
        idx = int(idx_str)
        joint = obs.get("joint", "?")
        expected = "hip" if idx % 2 == 0 else "knee"
        if joint not in (expected, "?", ""):
            verdict["anomalies"].append(
                f"Hip/knee swap: index {idx} (expected {expected}, observed {joint}) — "
                f"fix: swap GPIO entries in servos.cpp:13 or use cmd_servo_pin"
            )

    # Pitch-based A/B classification
    pitch = transforms.get("pitch", {})
    extended_legs = set(pitch.get("extended", []))
    physical_front = {"FL", "FR"}
    physical_rear = {"RL", "RR"}

    front_leg_extended = len(extended_legs & physical_front) > len(extended_legs & physical_rear)
    rear_leg_extended = not front_leg_extended and len(extended_legs & physical_rear) > 0

    # Under pitch+12 (nose up), physical front legs should extend.
    if front_leg_extended:
        # Correct mechanical response. Now check WHICH code indices those are.
        front_indices = set()
        for leg in physical_front:
            front_indices.update(leg_to_indices.get(leg, []))

        scenario_a_front = {0, 1, 4, 5}  # ik.h legs 0+2 = indices {0,1} + {4,5}
        scenario_b_front = {0, 1, 2, 3}  # standard front legs = first 4 indices

        overlap_a = len(front_indices & scenario_a_front)
        overlap_b = len(front_indices & scenario_b_front)

        verdict["pitch_analysis"] = {
            "physical_front_extended": True,
            "physical_front_code_indices": sorted(front_indices),
            "overlap_with_scenario_A": overlap_a,
            "overlap_with_scenario_B": overlap_b,
        }

        if overlap_a > overlap_b:
            verdict["scenario"] = "A"
            verdict["confidence"] = "high" if overlap_a >= 3 else "medium"
            verdict["interpretation"] = (
                "Wiring matches ik.h's non-standard scheme. The transform engine is correct. "
                "'FR' means physical rear-left and 'RL' means physical front-right in this codebase. "
                "Action: fix docs/hardware.md, docs/protocol.md, and mock/physics.cpp:73-76 "
                "to use non-standard labels or rename consistently."
            )
        elif overlap_b > overlap_a:
            verdict["scenario"] = "B"
            verdict["confidence"] = "high" if overlap_b >= 3 else "medium"
            verdict["interpretation"] = (
                "Wiring matches STANDARD naming (FR=front-right, RL=rear-left), but ik.h's "
                "spatial code maps leg 1 to physical rear-left and leg 2 to physical front-right. "
                "The transform engine commands wrong legs under pitch/roll/yaw. "
                "Action: fix ik.h:167-168 and cal_table() at ik.h:98-104 to swap the X/Y groupings "
                "for legs 1 and 2, and audit mock/physics.cpp:73-76."
            )
        else:
            verdict["scenario"] = "AMBIGUOUS"
            verdict["confidence"] = "low"
            verdict["interpretation"] = (
                "Could not determine scenario — Step 0 tagging may be incomplete or pitch "
                "observation was unclear. Re-run with clearer tape tags."
            )
    else:
        verdict["pitch_analysis"] = {
            "physical_front_extended": False,
            "extended_legs_observed": sorted(extended_legs),
        }
        verdict["anomalies"].append(
            "Pitch+12 did NOT produce physical-front extension — either the observation was "
            "inverted (rear extended instead) or the pitch command was sent with wrong direction. "
            "Re-run with careful observation; if rear extended under pitch+12 (nose-up), note this."
        )

    return verdict


def print_verdict(verdict: dict) -> None:
    print("\n" + "=" * 60)
    print("VERDICT")
    print("=" * 60)
    print(f"  Scenario:    {verdict['scenario']}")
    print(f"  Confidence:  {verdict['confidence']}")
    if "interpretation" in verdict:
        print(f"\n  {verdict['interpretation']}")

    print(f"\n  Step 0 mapping:")
    for leg, indices in verdict["step0_summary"]["leg_to_indices"].items():
        print(f"    {leg} → code indices {indices}")
    dead = verdict["step0_summary"]["dead_or_unknown"]
    if dead:
        print(f"    DEAD/UNKNOWN → {dead}")

    if verdict["anomalies"]:
        print(f"\n  Anomalies detected:")
        for a in verdict["anomalies"]:
            print(f"    • {a}")
    else:
        print("\n  No per-index anomalies detected.")


async def main(port: str | None, tcp_host: str | None, tcp_port: int) -> int:
    if tcp_host:
        dog = Dog(host=tcp_host, tcp_port=tcp_port)
        transport = f"TCP {tcp_host}:{tcp_port}"
    else:
        port = port or find_serial_port()
        if not port:
            print("ERROR: no USB serial port found. Pass --port /dev/cu.usbserial-XXXX")
            return 1
        dog = Dog(port=port)
        transport = f"USB {port}"

    print(f"[identify] connecting via {transport}")

    if not await setup(dog):
        await dog.close()
        return 1

    try:
        step0 = await run_step0(dog)
        transforms = await run_transform_steps(dog)
    finally:
        await teardown(dog)

    # Convert int keys to strings for JSON serialisation
    step0_serialisable = {str(k): v for k, v in step0.items()}

    verdict = compute_verdict(step0_serialisable, transforms)
    print_verdict(verdict)

    output = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "transport": transport,
        "step0": step0_serialisable,
        "transforms": transforms,
        "verdict": verdict,
    }

    out_path = REPO_ROOT / "host" / "servo_mapping.json"
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\n[identify] saved results to {out_path}")

    return 0 if verdict["scenario"] != "UNDETERMINED" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Servo identification diagnostic")
    parser.add_argument("--port", default=None, help="USB serial port (auto-detect if omitted)")
    parser.add_argument("--tcp", default=None, metavar="HOST",
                        help="Connect via TCP instead of serial (e.g. 192.168.1.x)")
    parser.add_argument("--tcp-port", type=int, default=9000, metavar="PORT",
                        help="TCP port (default: 9000)")
    args = parser.parse_args()

    sys.exit(asyncio.run(main(args.port, args.tcp, args.tcp_port)))
