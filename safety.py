"""
safety.py — PRECOG-Edge Step 4
Consequence Reasoning: predictive collision detection against a protected zone.

The SafetyMonitor checks whether ANY predicted future position intersects
the protected zone — flagging danger BEFORE the event, not after.

This is the core concept behind:
  - Aircraft TCAS (Traffic Collision Avoidance System)
  - Industrial robot workspace monitoring
  - Missile intercept prediction
"""

import math
import cv2
import numpy as np
from dataclasses import dataclass


@dataclass
class CollisionEvent:
    """Details about a predicted collision."""
    track_id:        int
    impact_frame:    int          # frames from now until first intersection
    impact_point:    tuple        # (x, y) of first intersection
    severity:        float        # 0.0 (grazing) → 1.0 (direct center hit)


class SafetyMonitor:
    """
    Monitors predicted trajectories against a circular protected zone.
    Supports multiple simultaneous tracks.
    """

    def __init__(self,
                 zone_center: tuple = (320, 240),
                 radius: int = 120):
        self.zone_center = zone_center
        self.radius      = radius

    # ── Core check ───────────────────────────────────────────────────────────

    def check_track(self,
                    track_id: int,
                    future_points: list[tuple[float, float]]
                    ) -> CollisionEvent | None:
        """
        Check one track's predicted trajectory against the zone.

        Returns a CollisionEvent if any future point is inside the zone,
        else None.
        """
        cx, cy = self.zone_center

        for i, (fx, fy) in enumerate(future_points):
            dist = math.hypot(fx - cx, fy - cy)
            if dist < self.radius:
                # How close to center? 0 = edge, 1 = bulls-eye
                severity = max(0.0, 1.0 - (dist / self.radius))
                return CollisionEvent(
                    track_id     = track_id,
                    impact_frame = i + 1,
                    impact_point = (int(fx), int(fy)),
                    severity     = severity,
                )
        return None

    def check_all(self,
                  tracks: dict[int, list[tuple[float, float]]]
                  ) -> list[CollisionEvent]:
        """
        Check multiple tracks at once.

        Args:
            tracks: {track_id: [future (x,y) points]}

        Returns:
            Sorted list of CollisionEvents (soonest first).
        """
        events = []
        for tid, pts in tracks.items():
            ev = self.check_track(tid, pts)
            if ev:
                events.append(ev)
        events.sort(key=lambda e: e.impact_frame)
        return events

    # ── Drawing helpers ───────────────────────────────────────────────────────

    def draw_zone(self, frame, events: list[CollisionEvent]):
        """
        Draw the protected zone circle.
        Color: yellow (safe) → orange → red (danger), pulses when hit.
        """
        cx, cy = self.zone_center

        if events:
            # Pick the most severe event for color
            worst   = max(events, key=lambda e: e.severity)
            urgency = worst.severity          # 0→1

            # Urgency-scaled color: yellow → red
            r = 255
            g = int(255 * (1.0 - urgency * 0.85))
            color     = (0, g, r)             # BGR
            thickness = 3

            # Draw a pulsing inner fill
            overlay = frame.copy()
            cv2.circle(overlay, (cx, cy), self.radius, color, -1)
            alpha = 0.12 + urgency * 0.12
            cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

            # Impact point marker
            cv2.circle(frame, worst.impact_point, 8, (0, 0, 255), -1)
            cv2.circle(frame, worst.impact_point, 12, (0, 0, 255), 2)

        else:
            color     = (0, 220, 220)         # cyan/teal = safe
            thickness = 2

        cv2.circle(frame, (cx, cy), self.radius, color, thickness)

        # Zone label
        label = "PROTECTED ZONE"
        tw, _  = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)[0], None
        cv2.putText(
            frame, label,
            (cx - 52, cy + self.radius + 16),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38,
            color, 1, cv2.LINE_AA
        )

    def draw_status(self, frame, events: list[CollisionEvent]):
        """
        Draw the main status banner at the top of the frame.
        """
        h_img, w_img = frame.shape[:2]

        if events:
            # ── DANGER ──────────────────────────────────────────────────────
            soonest = events[0]
            bg      = (0, 0, 180)
            text    = (f"⚠ PREDICTED COLLISION — T{soonest.track_id} "
                       f"in {soonest.impact_frame} frames  "
                       f"| ACTION BLOCKED")
            txt_color = (0, 50, 255)

            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 36), (w_img, 70), bg, -1)
            cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
            cv2.putText(
                frame, text,
                (8, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 80, 80), 2, cv2.LINE_AA
            )
        else:
            # ── SAFE ────────────────────────────────────────────────────────
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 36), (w_img, 70), (0, 60, 0), -1)
            cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
            cv2.putText(
                frame, "✔  SAFE TO OPERATE — no predicted intersection",
                (8, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 80), 1, cv2.LINE_AA
            )
