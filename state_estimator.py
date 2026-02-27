"""
state_estimator.py — PRECOG-Edge Step 2
Per-object Kalman filter tracking.

State vector per object:  [x, y, vx, vy]
  x, y   → position (pixels)
  vx, vy → velocity (pixels/frame)

The Kalman filter:
  • Predict → where should the object be now given its last velocity?
  • Update  → correct with the new measured position
This gives us smooth, noise-resistant position AND derived velocity.
"""

import numpy as np


class ObjectKalman:
    """Single-object 4-state Kalman filter (x, y, vx, vy)."""

    def __init__(self, cx: float, cy: float):
        dt = 1.0  # one frame time-step

        # State transition: x_{k} = F * x_{k-1}
        self.F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1],
        ], dtype=float)

        # Observation matrix: we only measure (x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # Process noise covariance (how much we trust the motion model)
        q = 1.0
        self.Q = np.eye(4, dtype=float) * q

        # Measurement noise covariance (how noisy is the detector?)
        r = 10.0
        self.R = np.eye(2, dtype=float) * r

        # Initial state and covariance
        self.x = np.array([[cx], [cy], [0.0], [0.0]], dtype=float)
        self.P = np.eye(4, dtype=float) * 500.0

        # Age tracking
        self.age         = 0   # frames since created
        self.missed      = 0   # consecutive frames with no match
        self.max_missed  = 10  # prune track after this many misses

    # ── Kalman steps ──────────────────────────────────────────────────────────

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.missed += 1
        self.age    += 1
        return self.x.flatten()

    def update(self, cx: float, cy: float):
        z = np.array([[cx], [cy]], dtype=float)

        S = self.H @ self.P @ self.H.T + self.R          # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)         # Kalman gain
        y = z - self.H @ self.x                           # innovation

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.missed = 0
        return self.x.flatten()

    # ── Accessors ─────────────────────────────────────────────────────────────

    @property
    def position(self):
        return float(self.x[0, 0]), float(self.x[1, 0])

    @property
    def velocity(self):
        return float(self.x[2, 0]), float(self.x[3, 0])

    @property
    def speed(self):
        vx, vy = self.velocity
        return float(np.hypot(vx, vy))

    @property
    def stale(self):
        return self.missed >= self.max_missed


# ── Multi-object manager ───────────────────────────────────────────────────────

class StateEstimator:
    """
    Maintains a pool of ObjectKalman filters.
    Matches incoming detections to existing tracks by nearest-centroid distance.
    """

    def __init__(self, max_distance: float = 80.0):
        self.tracks:        list[ObjectKalman] = []
        self.max_distance   = max_distance
        self._next_id       = 0
        self.track_ids:     list[int] = []

    def update(self, detections: list[tuple]) -> list[dict]:
        """
        Args:
            detections: list of (cx, cy, w, h) from MotionTracker

        Returns:
            list of dicts with keys:
              id, cx, cy, vx, vy, speed, w, h
        """

        # 1. Predict all existing tracks forward
        for t in self.tracks:
            t.predict()

        # 2. Match detections to tracks (greedy nearest-neighbour)
        matched_track_idx = set()
        matched_det_idx   = set()

        for di, (cx, cy, w, h) in enumerate(detections):
            best_dist  = self.max_distance
            best_tidx  = -1
            for ti, track in enumerate(self.tracks):
                if ti in matched_track_idx:
                    continue
                tx, ty = track.position
                dist = np.hypot(cx - tx, cy - ty)
                if dist < best_dist:
                    best_dist = dist
                    best_tidx = ti

            if best_tidx >= 0:
                self.tracks[best_tidx].update(cx, cy)
                matched_track_idx.add(best_tidx)
                matched_det_idx.add(di)

        # 3. Birth new tracks for unmatched detections
        for di, (cx, cy, w, h) in enumerate(detections):
            if di not in matched_det_idx:
                self.tracks.append(ObjectKalman(cx, cy))
                self.track_ids.append(self._next_id)
                self._next_id += 1

        # 4. Prune stale tracks
        alive = [(t, tid) for t, tid in zip(self.tracks, self.track_ids) if not t.stale]
        self.tracks    = [a[0] for a in alive]
        self.track_ids = [a[1] for a in alive]

        # 5. Build detection-size lookup (last seen w/h per track)
        det_map = {(round(cx), round(cy)): (w, h) for (cx, cy, w, h) in detections}

        # 6. Return state dicts for all live tracks
        results = []
        for track, tid in zip(self.tracks, self.track_ids):
            px, py = track.position
            vx, vy = track.velocity
            # find closest detection for w/h
            best_wh = (40, 40)
            best_d  = 9999
            for (dcx, dcy), (dw, dh) in det_map.items():
                d = np.hypot(px - dcx, py - dcy)
                if d < best_d:
                    best_d  = d
                    best_wh = (dw, dh)

            results.append({
                "id":    tid,
                "cx":    px,
                "cy":    py,
                "vx":    vx,
                "vy":    vy,
                "speed": track.speed,
                "w":     best_wh[0],
                "h":     best_wh[1],
                "age":   track.age,
            })

        return results
