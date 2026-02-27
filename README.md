# PRECOG EDGE — Predictive Physical Intelligence Engine

> **"We are not improving perception. We are giving machines foresight."**

A proof-of-concept for a new computing layer: **Reactive AI → Preventive AI**.

Instead of detecting collisions, PRECOG EDGE predicts them — and acts before they happen.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        PRECOG EDGE PIPELINE                         │
│                                                                     │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────┐    ┌───────┐  │
│  │ SENSOR   │───▶│    STATE     │───▶│  TRAJECTORY │───▶│SAFETY │  │
│  │ LAYER    │    │  ESTIMATION  │    │  PREDICTION │    │ENGINE │  │
│  │          │    │              │    │             │    │       │  │
│  │ Webcam   │    │  Kalman      │    │ Quadratic   │    │ Zone  │  │
│  │ OpenCV   │    │  Filter      │    │ Extrapolat. │    │ Check │  │
│  │ MOG2 BG  │    │  [x,y,vx,vy]│    │ 40 frames   │    │       │  │
│  └──────────┘    └──────────────┘    └─────────────┘    └───┬───┘  │
│       │                │                    │               │       │
│  "Where is it?"  "How is it moving?"  "Where will it be?" "Safe?" │
│                                                             │       │
│                                              ┌──────────────▼────┐  │
│                                              │  ROBOT ACTUATOR   │  │
│                                              │  Danger → FREEZE  │  │
│                                              │  Safe   → MOVE    │  │
│                                              └───────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘

  AMD Mapping:
  FPGA  → Sensor timing / deterministic capture
  GPU   → Physics simulation / trajectory math
  Ryzen AI → Kalman prediction / state estimation
  CPU   → Safety control logic
```

---

## What It Does

| Layer | File | What it proves |
|---|---|---|
| Sensor ingestion | `tracker.py` | Real-world perception via MOG2 background subtraction |
| State estimation | `state_estimator.py` | Kalman filter → position + velocity per object |
| Trajectory prediction | `predictor.py` | Quadratic extrapolation → 40 frames into the future |
| Safety decision | `safety.py` | Collision detected **before** impact occurs |
| Robot actuator | `main.py` | Physical action suppressed by predicted danger |

---

## Key Insight

Every other system:
```
event happens → AI reacts
```

PRECOG EDGE:
```
event predicted → AI prevents
```

---

## How to Run

### Requirements
```bash
pip install opencv-python numpy scipy matplotlib
```

### Run
```bash
python main.py
```

### Controls
| Key | Action |
|---|---|
| `ESC` / `Q` | Quit |
| `M` | Toggle motion mask window |
| `R` | Reset robot arm position |

---

## Demo Guide

1. **Start** — robot bar moves freely across the bottom of the frame
2. **Roll an object** toward the **cyan circle** (protected zone) in the center
3. The **cyan prediction arc** enters the circle before the object does
4. System fires: `⚠ PREDICTED COLLISION — ACTION BLOCKED`
5. **Robot freezes** — stopped by prediction, not by contact
6. Remove object → robot resumes immediately

> **"The robot did not stop because it saw a collision.  
> It stopped because it predicted a collision."**

---

## On-Screen Legend

| Color | Meaning |
|---|---|
| 🟩 Green box | Detected object |
| 🟧 Orange arrow | Velocity estimate (direction + magnitude) |
| 🩵 Cyan dots | Predicted future trajectory (40 frames) |
| ⭕ Teal circle | Protected robot workspace |
| 🔴 Red fill | Predicted collision imminent |

---

## File Structure

```
precog-edge/
 ├── main.py             ← Entry point, all layers wired
 ├── tracker.py          ← Step 1: MOG2 motion detection
 ├── state_estimator.py  ← Step 2: Multi-object Kalman filter
 ├── predictor.py        ← Step 3: Quadratic trajectory prediction
 ├── safety.py           ← Step 4: Predictive collision detection
 └── requirements.txt
```

---

## AMD Heterogeneous Computing Alignment

This architecture maps directly to AMD's compute stack:

- **FPGA** — deterministic sensor capture timing (represented by MOG2 pipeline)
- **GPU** — parallel physics trajectory computation
- **Ryzen AI NPU** — Kalman filter inference at the edge
- **CPU** — real-time safety logic + actuation control

PRECOG EDGE demonstrates a **predictive cognition layer** portable to any AMD heterogeneous platform.
