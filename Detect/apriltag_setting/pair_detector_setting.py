from collections import deque
from typing import List, Dict, Any, Optional, Tuple

import cv2
import numpy as np


class PairDetector:
    def __init__(self, history_len: int = 6, stable_threshold: int = 4):
        self.history_len = history_len
        self.stable_threshold = stable_threshold
        self._history: Dict[str, deque] = {}

    def clear_history(self) -> None:
        self._history.clear()

    def _avg_rotation_two(self, R1: np.ndarray, R2: np.ndarray) -> np.ndarray:
        # Average two rotation matrices via SVD of sum, ensure det=+1
        R_sum = R1 + R2
        U, _, Vt = np.linalg.svd(R_sum)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt
        return R

    def _combine_pose(self, A: Dict[str, Any], B: Dict[str, Any]) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        ta, tb = A.get("t"), B.get("t")
        Ra, Rb = A.get("R"), B.get("R")

        if ta is not None and tb is not None:
            t_comb = (ta + tb) / 2.0
        else:
            t_comb = ta if ta is not None else tb  # fall back to whichever is available

        R_comb = None
        if Ra is not None and Rb is not None:
            try:
                R_comb = self._avg_rotation_two(Ra, Rb)
            except Exception:
                pass

        return R_comb, t_comb

    def update_and_detect(self, results: List[Any]) -> List[Dict[str, Any]]:
        tag_infos: List[Dict[str, Any]] = []
        for r in results:
            tag_id = int(r.tag_id)
            try:
                center = r.center.copy()
            except Exception:
                try:
                    center = np.mean(r.corners, axis=0)
                except Exception:
                    center = np.array([0.0, 0.0])
            R = r.pose_R.copy() if hasattr(r, "pose_R") else None
            t = None
            if hasattr(r, "pose_t"):
                try:
                    t = r.pose_t.reshape(3).copy()
                except Exception:
                    pass
            tag_infos.append({"id": tag_id, "center": center, "R": R, "t": t})

        detected_pairs: List[Dict[str, Any]] = []
        n = len(tag_infos)
        for i in range(n):
            for j in range(i + 1, n):
                A, B = tag_infos[i], tag_infos[j]
                idA, idB = A["id"], B["id"]
                key = f"{min(idA, idB)}_{max(idA, idB)}"

                if A["t"] is not None and B["t"] is not None:
                    left_id = idA if A["t"][0] < B["t"][0] else idB
                else:
                    left_id = idA if float(A["center"][0]) < float(B["center"][0]) else idB

                other_id  = idA + idB - left_id
                relation  = f"{left_id}_{other_id}"
                direction = "going" if relation == key else "returning"

                dq = self._history.setdefault(key, deque(maxlen=self.history_len))
                dq.append(direction)
                stable = sum(1 for v in dq if v == direction) >= self.stable_threshold

                cx = int((float(A["center"][0]) + float(B["center"][0])) / 2.0)
                cy = int((float(A["center"][1]) + float(B["center"][1])) / 2.0)

                R_comb, t_comb = self._combine_pose(A, B)

                detected_pairs.append({
                    "key":      key,
                    "relation": relation,
                    "direction": direction,
                    "center":   (cx, cy),
                    "members":  (idA, idB),
                    "stable":   stable,
                    "R":        R_comb,
                    "t":        t_comb,
                    "_tag_infos": {A["id"]: A, B["id"]: B},  # expose for subclasses
                })

        return detected_pairs


def draw_pair_labels(img: np.ndarray, pairs: List[Dict[str, Any]],
                     color_stable:   Tuple[int, int, int] = (255, 0, 255),
                     color_unstable: Tuple[int, int, int] = (200, 120, 255)) -> None:
    LINE_HEIGHT  = 24
    PIXEL_OFFSET = 100
    TEXT_X       = 8
    for p in pairs:
        cx, cy = p["center"]
        color  = color_stable if p.get("stable", False) else color_unstable
        cv2.circle(img, (cx, cy), 5, color, -1)

        t_vec = p.get("t")
        distance_text = f"Dist={float(np.linalg.norm(t_vec)):.3f} m" if t_vec is not None else ""

        base_y = cy + PIXEL_OFFSET
        cv2.putText(img, f"id: {p['key']}",
                    (cx + TEXT_X, base_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        cv2.putText(img, f"tag relation={p['relation']} {p['direction']}",
                    (cx + TEXT_X, base_y + LINE_HEIGHT),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 255), 2, cv2.LINE_AA)
        if distance_text:
            cv2.putText(img, distance_text,
                        (cx + TEXT_X, base_y + LINE_HEIGHT * 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 2, cv2.LINE_AA)


def draw_axes(img, K, rvec, tvec, length=0.03):
    try:
        axis = np.float32([[0,0,0],[length,0,0],[0,length,0],[0,0,length]]).reshape(-1, 3)
        pts, _ = cv2.projectPoints(axis, rvec, tvec, K, np.zeros((5, 1), np.float32))
        pts = pts.reshape(-1, 2).astype(int)
        o, x, y, z = pts
        cv2.line(img, tuple(o), tuple(x), (0,   0, 255), 2)
        cv2.line(img, tuple(o), tuple(y), (0, 255,   0), 2)
        cv2.line(img, tuple(o), tuple(z), (255, 0,   0), 2)
    except Exception:
        pass
