"""
main.py — PRECOG-Edge  Steps 1 + 2 + 3 + 4 + 5 + 6 (FINAL)
Sensor → State Estimation → Prediction → Safety Decision → Robot Actuator → Demo UI

Controls:
  ESC / Q  → quit
  M        → toggle Motion Mask window
  R        → reset robot to start position
"""

import cv2
import numpy as np
from tracker         import MotionTracker
from state_estimator import StateEstimator
from predictor       import TrajectoryPredictor, draw_trajectory
from safety          import SafetyMonitor

# ── Configuration ─────────────────────────────────────────────────────────────
CAMERA_INDEX  = 0
SPEED_HIGH    = 8.0
VEL_SCALE     = 5
HORIZON       = 40
ZONE_RADIUS   = 120
ROBOT_SPEED   = 3
# ──────────────────────────────────────────────────────────────────────────────


def speed_color(speed):
    t = min(speed / SPEED_HIGH, 1.0)
    return (0, int(255 * (1 - t * 0.7)), int(255 * t))


def draw_tracks(frame, states):
    for s in states:
        cx, cy = int(s["cx"]), int(s["cy"])
        w, h   = int(s["w"]), int(s["h"])
        color  = speed_color(s["speed"])

        cv2.rectangle(frame,
                      (cx - w//2, cy - h//2),
                      (cx + w//2, cy + h//2), color, 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        ex = int(cx + s["vx"] * VEL_SCALE)
        ey = int(cy + s["vy"] * VEL_SCALE)
        cv2.arrowedLine(frame, (cx, cy), (ex, ey), (255, 165, 0), 2, tipLength=0.3)

        cv2.putText(frame, f"T{s['id']}  {s['speed']:.1f}px/f",
                    (cx - w//2, max(cy - h//2 - 6, 80)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1, cv2.LINE_AA)


def draw_robot(frame, robot_x, robot_y, danger):
    ARM_W, ARM_H = 80, 18
    color  = (60, 60, 200) if danger else (200, 110, 0)

    cv2.rectangle(frame,
                  (robot_x, robot_y - ARM_H),
                  (robot_x + ARM_W, robot_y + ARM_H),
                  color, -1)
    cv2.rectangle(frame,
                  (robot_x, robot_y - ARM_H),
                  (robot_x + ARM_W, robot_y + ARM_H),
                  (255, 255, 255), 1)

    status = "⛔ FROZEN" if danger else "▶ MOVING"
    cv2.putText(frame, f"ROBOT ARM  {status}",
                (robot_x, robot_y - ARM_H - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (80, 80, 255) if danger else (0, 200, 200),
                1, cv2.LINE_AA)

    if not danger:
        cv2.arrowedLine(frame,
                        (robot_x + ARM_W, robot_y),
                        (robot_x + ARM_W + 24, robot_y),
                        (0, 220, 220), 2, tipLength=0.4)


def draw_title_banner(frame):
    """Top title — makes it look like a real product prototype."""
    h_img, w_img = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w_img, 36), (10, 10, 30), -1)
    cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

    cv2.putText(frame,
                "PRECOG EDGE  —  Predictive Physical Intelligence Engine",
                (14, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)


def draw_prediction_timer(frame, n_future_points: int):
    """Show how far ahead the system is forecasting."""
    ms = int(n_future_points * 33)   # ~30 FPS → 33 ms per frame
    cv2.putText(frame,
                f"Forecasting  {ms} ms  into the future",
                (14, 62),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 200, 255), 1, cv2.LINE_AA)


def draw_legend(frame):
    """Bottom legend — judges understand every element without asking."""
    h_img, w_img = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h_img - 110), (w_img, h_img), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    items = [
        ((0,  255, 0),   "GREEN box  =  Detected object"),
        ((255, 165, 0),  "ORANGE arrow  =  Velocity estimate"),
        ((0,  255, 255), "CYAN dots  =  Predicted future trajectory"),
        ((0,  220, 220), "TEAL circle  =  Protected robot workspace"),
    ]

    for i, (color, text) in enumerate(items):
        y = h_img - 90 + i * 22
        cv2.circle(frame, (18, y - 4), 6, color, -1)
        cv2.putText(frame, text, (32, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, color, 1, cv2.LINE_AA)


def draw_status_bar(frame, states, events):
    """Second bar — live telemetry snapshot."""
    h_img, w_img = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 36), (w_img, 72), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

    if events:
        soonest = events[0]
        text    = (f"⚠  PREDICTED COLLISION — Track T{soonest.track_id} "
                   f"in {soonest.impact_frame} frames  |  ACTION BLOCKED")
        color   = (80, 80, 255)
    else:
        text  = "✔  SAFE TO OPERATE  —  no trajectory intersects protected zone"
        color = (0, 210, 80)

    cv2.putText(frame, text, (14, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)


def main():
    SHOW_MASK = True

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Camera {CAMERA_INDEX} not found. Try CAMERA_INDEX 1 or 2.")
        return

    ret, test_frame = cap.read()
    h_frame = test_frame.shape[0] if ret else 480
    w_frame = test_frame.shape[1] if ret else 640
    zone_center = (w_frame // 2, h_frame // 2)

    tracker    = MotionTracker()
    estimator  = StateEstimator(max_distance=80.0)
    safety     = SafetyMonitor(zone_center=zone_center, radius=ZONE_RADIUS)
    predictors: dict[int, TrajectoryPredictor] = {}

    robot_x   = 20
    robot_y   = h_frame - 55
    robot_dir = 1

    total_future_pts = 0   # for prediction timer

    print("[INFO] PRECOG EDGE — Full demo active.")
    print("[INFO] ESC/Q=quit  M=mask  R=reset robot")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # ── Step 1: Detection ─────────────────────────────────────────────────
        detections, mask = tracker.detect(frame)

        # ── Step 2: State estimation ──────────────────────────────────────────
        states = estimator.update(detections)

        # ── Step 3: Trajectory prediction ─────────────────────────────────────
        track_futures: dict[int, list] = {}
        total_future_pts = HORIZON      # always show full horizon on timer

        for s in states:
            tid = s["id"]
            if tid not in predictors:
                predictors[tid] = TrajectoryPredictor(history_len=8, horizon=HORIZON)
            future = predictors[tid].update_and_predict(
                s["cx"], s["cy"], s["vx"], s["vy"]
            )
            track_futures[tid] = future
            draw_trajectory(frame, future)

        for dead in [k for k in predictors if k not in {s["id"] for s in states}]:
            del predictors[dead]

        # ── Step 4: Safety decision ───────────────────────────────────────────
        events = safety.check_all(track_futures)
        danger = len(events) > 0

        safety.draw_zone(frame, events)

        # ── Step 5: Robot actuator ────────────────────────────────────────────
        if not danger:
            robot_x += ROBOT_SPEED * robot_dir
            if robot_x + 80 >= w_frame - 10:
                robot_dir = -1
            elif robot_x <= 10:
                robot_dir = 1

        draw_robot(frame, robot_x, robot_y, danger)

        # ── Step 6: Demo UI overlays ──────────────────────────────────────────
        draw_tracks(frame, states)
        draw_title_banner(frame)
        draw_prediction_timer(frame, total_future_pts)
        draw_status_bar(frame, states, events)
        draw_legend(frame)

        cv2.imshow("PRECOG EDGE", frame)
        if SHOW_MASK:
            cv2.imshow("Motion Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        elif key == ord("m"):
            SHOW_MASK = not SHOW_MASK
            if not SHOW_MASK:
                cv2.destroyWindow("Motion Mask")
        elif key == ord("r"):
            robot_x, robot_dir = 20, 1

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] PRECOG EDGE shut down.")


if __name__ == "__main__":
    main()
