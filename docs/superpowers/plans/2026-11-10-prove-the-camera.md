# Prove the Camera — Stage 1 Runbook

**Goal:** decide, with one lap of a real room, whether the dog's own camera plus a feed-forward geometry model gives a map and poses good enough to navigate on. If yes, the perception roadmap in `docs/overview.md` continues camera-only. If no, a 2D LiDAR is fitted.

**State (2026-11-10):** all software for this stage is written. Capture is tested against the mock firmware with a stand-in camera. The reconstruction geometry is tested on a synthetic room, and the model wrapper was run on CPU with the small model. The camera firmware compiles. **Nothing here has run on the dog or its camera yet** — the steps below are that first run.

---

## 0. One-time setup on the GPU machine

```bash
pip install -e ".[perception]"
```

```bash
pip install torch torchvision xformers
```

```bash
git clone https://github.com/ByteDance-Seed/Depth-Anything-3 && pip install -e Depth-Anything-3
```

```bash
python -c "import torch; print(torch.cuda.is_available())"
```

That last line must print `True`. On Windows, install torch from the CUDA index given at pytorch.org first; the default wheel there is CPU-only.

On a machine that has never built the firmware, `make bark-mock` needs the PlatformIO libraries once:

```bash
cd firmware && pio pkg install
```

## 1. Flash and check the camera

The vision module needs `firmware/include/config_local.h` (same WiFi credentials as the dog). Plug the module in by its own USB-C port.

```bash
bark flash camera
```

Add `--serial PORT` if the dog is plugged in too. A wrong port fails safely: the flasher checks the chip type.

```bash
pio device monitor -d camera
```

It prints the URL once it joins WiFi. Then, in a browser:

- `http://bark-cam.local/` — status text. Note the sensor PID and add it to `docs/hardware.md`.
- `http://bark-cam.local/capture` — a still. It must be upright and in focus.

If `bark-cam.local` does not resolve, use the IP address from the monitor.

If the module does not boot or the image is wrong, keep the serial output. Hiwonder's stock firmware and flash tool are linked from the appendix of their wiki (see `docs/hardware.md`) to restore it.

## 2. Measure the camera height

Engage the dog so it stands. Measure from the floor to the centre of the lens, in millimetres. This one number sets the scale of the whole map; add it to `docs/hardware.md`.

## 3. Record one lap

```bash
bark --cam-url http://bark-cam.local/capture --record
```

Drive from the web UI:

1. Engage. Wait for the chirp — that is keyframe 0.
2. Walk about one body length, release, **wait for the chirp**, repeat.
3. Turn in steps of 30° or less, releasing and waiting for the chirp after each.
4. Go once around the room and finish where you started, facing the same way.
5. Stop the server with Ctrl-C.

Aim for 40–60 keyframes. Each view should still show most of what the previous one did. Keep the lights on and people and pets out of view.

The run lands in `runs/<date_time>/`. Check a few stills in `frames/` for blur before going on.

## 4. Reconstruct

```bash
bark reconstruct --cam-height-mm 105
```

Use your measured height. It picks the newest run; pass a run directory to choose another.

- Out of GPU memory: add `--max-frames 30`, or `--model depth-anything/DA3-BASE`.
- Plenty of GPU memory: `--model depth-anything/DA3NESTED-GIANT-LARGE-1.1` is the strongest model, and it predicts metres directly, so "Metric scale" should then come out near 1.0 — a third scale check.

Outputs are in `runs/<run>/recon/`: `map.png`, `cloud.ply` (open in MeshLab or CloudCompare), `report.json`.

## 5. Go / no-go

The thresholds are a first judgment, not validated numbers. The map picture outweighs any single figure.

| Check | Go | What it tells you |
|-------|----|-------------------|
| `map.png` | Recognisably the room; walls straight; first and last keyframes within ~10 cm | Poses and depth are globally consistent |
| Camera height spread | under 5% | Every keyframe sits at the same height above the fitted floor |
| Sonar vs camera | 0.8 – 1.25 | Two independent metric anchors agree |
| Heading vs gyro | under 10° RMS | Visual rotation matches the gyro over single moves |
| Forward speed spread | under 20% | One second of walking is repeatable enough for dead reckoning between keyframes |

A marginal result is not a no-go. First try the larger model, more overlap between keyframes, and better light.

**Go:** continue with stage 2 in `docs/overview.md` (simulated world), then run this same pipeline incrementally as a host behavior. The forward speed and turn rate from the report become the dead-reckoning constants.

**No-go:** fit a 2D LiDAR (LD19 class: 47 g, 5 V, one-way 230400-baud UART) to a spare UART pin on the J5 header, relay scans through the firmware, and localize by scan matching. The camera then serves semantics only. The capture recorder and run format stay as they are.

## After the run

Whatever the outcome, commit the report summary and the measured values into `docs/hardware.md`, and delete this file once stage 1 is closed (plans are pruned when complete).
