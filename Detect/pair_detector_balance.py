from typing import Any, Dict, List, Optional

import numpy as np

from pair_detector_setting import PairDetector


class BalancePairDetector(PairDetector):
    """Extends PairDetector to also expose per-tag translation vectors.

    Each pair dict gains two extra keys:
      t_left  — np.ndarray(3,) translation of the left tag  (camera-space x < right)
      t_right — np.ndarray(3,) translation of the right tag
    Both are None when pose is unavailable for either tag.
    Depth difference:  depth_diff = t_right[2] - t_left[2]
      > 0  → right tag is farther  → robot is angled left  → need to turn right
      < 0  → left  tag is farther  → robot is angled right → need to turn left
      ≈ 0  → robot faces tags perpendicularly
    """

    def update_and_detect(self, results: List[Any]) -> List[Dict[str, Any]]:
        pairs = super().update_and_detect(results)

        # Build a quick id→t lookup from the raw results
        id_to_t: Dict[int, Optional[np.ndarray]] = {}
        for r in results:
            tag_id = int(r.tag_id)
            t = None
            if hasattr(r, "pose_t"):
                try:
                    t = r.pose_t.reshape(3).copy()
                except Exception:
                    pass
            id_to_t[tag_id] = t

        for p in pairs:
            id_a, id_b = p["members"]
            ta = id_to_t.get(id_a)
            tb = id_to_t.get(id_b)

            if ta is not None and tb is not None:
                # left/right determined by camera-space x (t[0])
                if ta[0] < tb[0]:
                    p["t_left"]  = ta
                    p["t_right"] = tb
                else:
                    p["t_left"]  = tb
                    p["t_right"] = ta
            else:
                p["t_left"]  = None
                p["t_right"] = None

        return pairs
