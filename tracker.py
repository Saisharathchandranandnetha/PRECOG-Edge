"""
tracker.py — PRECOG-Edge Step 1
Motion detection using background subtraction (MOG2).
Conceptually models the FPGA sensor preprocessing layer:
deterministic object detection without a neural network.
"""

import cv2


class MotionTracker:
    def __init__(self):
        # MOG2 = Mixture of Gaussians v2 background subtractor
        # history=500 → frames to model background
        # varThreshold=25 → sensitivity (lower = more sensitive)
        self.bg = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=25, detectShadows=True
        )

    def detect(self, frame):
        """
        Apply background subtraction and find moving object blobs.

        Returns:
            objects : list of (cx, cy, w, h) for each detected blob
            mask    : binary mask image (useful for debug display)
        """
        mask = self.bg.apply(frame)

        # Remove shadows (gray pixels → 127) to get binary foreground only
        _, mask = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)

        # Morphological cleanup: remove small noise, fill gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)  # fill gaps

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        objects = []
        for c in contours:
            if cv2.contourArea(c) > 800:          # ignore tiny blobs
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2
                cy = y + h // 2
                objects.append((cx, cy, w, h))

        return objects, mask
