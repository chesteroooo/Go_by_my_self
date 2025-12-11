import math
from collections import deque
from typing import List, Dict, Any, Optional, Tuple

import cv2
import numpy as np

# Simple pair detector for AprilTag results from pupil_apriltags.Detector.detect(...)
# Usage:
#   pd = PairDetector(history_len=6, stable_threshold=4)
#   pairs = pd.update_and_detect(results)
# returns list of dicts with keys:
#   "key" (min_max id string),
#   "relation" (left_right id string, e.g. "0_1" or "1_0"),
#   "direction" ("going" or "returning"),
#   "center" (pixel tuple),
#   "members" (tuple of two ints),
#   "stable" (bool),
#   "R" (3x3 np.array or None),
#   "t" (3-d np.array or None)
class PairDetector:
    def __init__(self, history_len: int = 6, stable_threshold: int = 4):
        self.history_len = history_len
        self.stable_threshold = stable_threshold
        self._history: Dict[str, deque] = {}  # key -> deque of recent direction strings

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
        # Combine two poses: average translation, avg rotation if both present
        Ra = A.get("R")
        Rb = B.get("R")
        ta = A.get("t")
        tb = B.get("t")
        t_comb = None
        R_comb = None
        if ta is not None and tb is not None:
            t_comb = (ta + tb) / 2.0
        if Ra is not None and Rb is not None:
            try:
                R_comb = self._avg_rotation_two(Ra, Rb)
            except Exception:
                R_comb = None
        return R_comb, t_comb

    def update_and_detect(self, results: List[Any]) -> List[Dict[str, Any]]:
        """
        Process a single-frame 'results' (list of Detection objects).
        Returns detected pair list with relation and stability info.
        """
        # build simple tag_infos
        tag_infos: List[Dict[str, Any]] = []
        for r in results:
            tag_id = int(r.tag_id)
            center = None
            try:
                center = r.center.copy()
            except Exception:
                # fallback: try corners mean
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
                    t = None
            tag_infos.append({"id": tag_id, "center": center, "R": R, "t": t})

        detected_pairs: List[Dict[str, Any]] = []
        n = len(tag_infos)
        for i in range(n):
            for j in range(i + 1, n):
                A = tag_infos[i]
                B = tag_infos[j]
                idA = A["id"]
                idB = B["id"]
                minid = min(idA, idB)
                maxid = max(idA, idB)
                key = f"{minid}_{maxid}"

                # choose left/right using camera-space x (t[0]) if available; else use pixel x
                left_id = None
                if A["t"] is not None and B["t"] is not None:
                    left_id = idA if A["t"][0] < B["t"][0] else idB
                else:
                    left_id = idA if float(A["center"][0]) < float(B["center"][0]) else idB

                # relation preserves the left->right order (e.g. "0_1" or "1_0")
                other_id = idA + idB - left_id  # sum trick to get the other id
                relation = f"{left_id}_{other_id}"

                # direction: if relation equals canonical key -> "going", else "returning"
                direction = "going" if relation == key else "returning"

                # update history for stability
                dq = self._history.get(key)
                if dq is None:
                    dq = deque(maxlen=self.history_len)
                    self._history[key] = dq
                dq.append(direction)
                stable_count = sum(1 for v in dq if v == direction)
                stable = stable_count >= self.stable_threshold

                # center pixel midpoint for drawing
                cx = int((float(A["center"][0]) + float(B["center"][0])) / 2.0)
                cy = int((float(A["center"][1]) + float(B["center"][1])) / 2.0)

                # combine pose (R,t) if possible
                R_comb, t_comb = self._combine_pose(A, B)

                detected_pairs.append({
                    "key": key,
                    "relation": relation,
                    "direction": direction,
                    "center": (cx, cy),
                    "members": (idA, idB),
                    "stable": stable,
                    "R": R_comb,
                    "t": t_comb,
                })

        return detected_pairs



# Convenience function for quick use without creating class instance
def detect_pairs_once(results: List[Any],
                      history: Optional[Dict[str, deque]] = None,
                      history_len: int = 6,
                      stable_threshold: int = 4) -> List[Dict[str, Any]]:
    pd = PairDetector(history_len=history_len, stable_threshold=stable_threshold)
    if history is not None:
        pd._history = history
    return pd.update_and_detect(results)


# Example: draw pair labels on an image (simple)
def draw_pair_labels(img: np.ndarray, pairs: List[Dict[str, Any]],
                                  color_stable: Tuple[int, int, int] = (255, 0, 255),
                                  color_unstable: Tuple[int, int, int] = (200, 120, 255)) -> None:
    
    # 計算兩行文字之間的垂直間距
    # 這裡使用 24 像素作為兩行文字的間距
    LINE_HEIGHT = 24
    PIXEL_OFFSET = 100
    TEXT_X_OFFSET = 8
    for p in pairs:
        # 1. 取得圓點的中心點座標
        cx, cy = p["center"]
        color = color_stable if p.get("stable", False) else color_unstable
        
        # 2. 繪製圓點 (保持在物件的原始中心點位置)
        cv2.circle(img, (cx, cy), 5, color, -1)
        
       # 3. 檢查是否有組合後的 pose 資訊
        t_vec = p.get("t")
        distance_text = ""
        # t_vec 是 3D 平移向量 (x, y, z)，距離是其長度 (歐幾里得範數)
        if t_vec is not None:
            distance_m = float(np.linalg.norm(t_vec))
        distance_text = f"Dist={distance_m:.3f} m"
                    # 由於 t_vec 是米 (m)，直接顯示。
        
        # 4. 繪製文字標籤：將 Y 座標向下偏移 PIXEL_OFFSET
        
        # 基準 Y 座標：圓點中心 Y 加上偏移量
        base_y_label = cy + PIXEL_OFFSET
        
        # --- 第一行文字 (id) ---
        # 放置在 (cx + 8, base_y_label)
        cv2.putText(img, f"id: {p['key']}", (cx + TEXT_X_OFFSET, base_y_label),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        
        # --- 第二行文字 (relation and direction) ---
        # 放置在第一行文字下方 LINE_HEIGHT 處
        second_line_y = base_y_label + LINE_HEIGHT
        cv2.putText(img, f"tag relation={p['relation']} {p['direction']}", 
        (cx + TEXT_X_OFFSET, second_line_y),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 255), 2, cv2.LINE_AA)
                
        # --- 第三行文字 (距離) ---
                # 僅在有 pose 資訊時繪製
        if distance_text:
            third_line_y = second_line_y + LINE_HEIGHT # 在第二行下方 LINE_HEIGHT 處
            cv2.putText(img, distance_text, 
                        (cx + TEXT_X_OFFSET, third_line_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 2, cv2.LINE_AA)
            

# === 請補上這段缺少的函式 ===
def draw_axes(img, K, rvec, tvec, length=0.03):
    try:
        axis = np.float32([[0,0,0],[length,0,0],[0,length,0],[0,0,length]]).reshape(-1,3)
        dist0 = np.zeros((5,1), np.float32)
        pts, _ = cv2.projectPoints(axis, rvec, tvec, K, dist0)
        pts = pts.reshape(-1,2).astype(int)
        o, x, y, z = pts
        cv2.line(img, tuple(o), tuple(x), (0,0,255), 2)
        cv2.line(img, tuple(o), tuple(y), (0,255,0), 2)
        cv2.line(img, tuple(o), tuple(z), (255,0,0), 2)
    except Exception:
        pass