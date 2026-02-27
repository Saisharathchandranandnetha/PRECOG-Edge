"""
predictor.py — PRECOG-Edge Step 3
Future trajectory prediction.

Two models:
  1. Linear   — constant velocity (good for short horizons)
  2. Quadratic — constant acceleration estimated from velocity history
                 (better for curved / decelerating motion)

Both return a list of (x, y) predicted positions for the next `horizon` frames.
"""

from collections import deque
import numpy as np


# ── Model 1: Constant-velocity linear extrapolation ───────────────────────────

def predict_linear(cx: float, cy: float,
                   vx: float, vy: float,
                   horizon: int = 30) -> list[tuple[float, float]]:
    """
    Straight-line prediction assuming constant velocity.
    Fast, low-latency, accurate for small horizons.
    """
    return [(cx + vx * t, cy + vy * t) for t in range(1, horizon + 1)]


# ── Model 2: Constant-acceleration quadratic extrapolation ────────────────────

class TrajectoryPredictor:
    """
    Per-track predictor that maintains a short velocity history to estimate
    acceleration, then extrapolates position quadratically.

        x(t) = x0 + vx*t + 0.5*ax*t²
        y(t) = y0 + vy*t + 0.5*ay*t²

    Falls back to linear if not enough history yet.
    """

    def __init__(self, history_len: int = 8, horizon: int = 30):
        self.horizon     = horizon
        self.history_len = history_len
        # velocity history for acceleration estimation
        self._vx_hist: deque[float] = deque(maxlen=history_len)
        self._vy_hist: deque[float] = deque(maxlen=history_len)

    def update_and_predict(self,
                           cx: float, cy: float,
                           vx: float, vy: float
                           ) -> list[tuple[float, float]]:
        """
        Call once per frame with the Kalman-estimated state.
        Returns horizon predicted (x, y) positions.
        """
        self._vx_hist.append(vx)
        self._vy_hist.append(vy)

        # Need ≥2 samples to estimate acceleration
        if len(self._vx_hist) < 2:
            return predict_linear(cx, cy, vx, vy, self.horizon)

        # Average acceleration over recent history
        vx_arr = np.array(self._vx_hist)
        vy_arr = np.array(self._vy_hist)
        ax = float(np.mean(np.diff(vx_arr)))
        ay = float(np.mean(np.diff(vy_arr)))

        # Clamp acceleration to dampen instability
        MAX_A = 2.0
        ax = np.clip(ax, -MAX_A, MAX_A)
        ay = np.clip(ay, -MAX_A, MAX_A)

        pts = []
        for t in range(1, self.horizon + 1):
            px = cx + vx * t + 0.5 * ax * t * t
            py = cy + vy * t + 0.5 * ay * t * t
            pts.append((px, py))

        return pts


# ── Drawing helper ────────────────────────────────────────────────────────────

def draw_trajectory(frame, points: list[tuple[float, float]],
                    color_start=(0, 255, 255),   # cyan near object
                    color_end=(0, 80, 200),       # dark-blue far out
                    dot_radius: int = 3,
                    step: int = 3):
    """
    Draw predicted trajectory as fading dots on frame.

    Args:
        points    : list of (x, y) in frame-order (nearest → furthest)
        color_start / color_end : BGR colors to interpolate between
        dot_radius : pixel radius of each dot
        step      : draw every Nth point (reduces visual clutter)
    """
    n = len(points)
    for i in range(0, n, step):
        t   = i / max(n - 1, 1)                   # 0 → 1
        r   = int(color_start[0] + t * (color_end[0] - color_start[0]))
        g   = int(color_start[1] + t * (color_end[1] - color_start[1]))
        b_c = int(color_start[2] + t * (color_end[2] - color_start[2]))
        alpha = max(0.15, 1.0 - t * 0.85)          # fade out
        radius = max(1, int(dot_radius * (1 - t * 0.6)))

        px, py = int(points[i][0]), int(points[i][1])
        h_img, w_img = frame.shape[:2]
        if 0 <= px < w_img and 0 <= py < h_img:
            cv2.circle(frame, (px, py), radius, (r, g, b_c), -1)


# Only import cv2 at usage time to avoid circular issues at module level
import cv2  # noqa: E402 (must stay after draw_trajectory def)
