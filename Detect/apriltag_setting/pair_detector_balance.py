from typing import Any, Dict, List, Optional

import numpy as np

from pair_detector_setting import PairDetector


class BalancePairDetector(PairDetector):
    """Extends PairDetector to expose per-tag translation vectors for each pair.

    Each pair dict gains:
      t_left  — translation of the left tag  (camera-space x is smaller)
      t_right — translation of the right tag
    Both are None when pose is unavailable for either tag.

    depth_diff = t_right[2] - t_left[2]
      > 0  → right tag farther  → robot angled left  → turn right
      < 0  → left tag farther   → robot angled right → turn left
      ≈ 0  → robot faces tags straight on
    """

    def update_and_detect(self, results: List[Any]) -> List[Dict[str, Any]]:
        pairs = super().update_and_detect(results)

        for p in pairs:
            tag_infos = p.pop("_tag_infos")  # consumed here, not exposed further
            ta = tag_infos[p["id_left"]]["t"]
            tb = tag_infos[p["id_right"]]["t"]

            if ta is not None and tb is not None:
                p["t_left"], p["t_right"] = ta, tb
            else:
                p["t_left"] = p["t_right"] = None

        return pairs
