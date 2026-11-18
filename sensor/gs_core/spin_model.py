"""Empirical spin estimate from ball speed and launch angle (fallback only).

Used when the image-based spin measurement is not trustworthy; the shot is
then flagged ``spin_measured = False``. The table holds typical tour-average
launch conditions per club; the estimate interpolates in (launch angle, speed)
and scales spin with speed for slower swings. Individual shots deviate from
this by 20-40 % (strike quality, ball, lie), and the spin axis cannot be
guessed at all (reported as 0).
"""
from __future__ import annotations

import numpy as np

# ball speed m/s, launch deg, spin rpm
_TABLE = np.array([
    [74.7, 10.9, 2686], [70.6, 9.2, 3655], [68.0, 9.4, 4350], [65.3, 10.2, 4437], [63.5, 10.4, 4630],
    [61.2, 11.0, 4836], [59.0, 12.1, 5361], [56.8, 14.1, 6231], [53.6, 16.3, 7097], [51.4, 18.1, 7998],
    [48.7, 20.4, 8647], [45.6, 24.2, 9304], [40.0, 28.0, 9500], [34.0, 32.0, 9800],
])


def empirical_spin_rpm(speed_mps: float, vla_deg: float) -> float:
    """Typical total spin for a shot with this ball speed and launch angle."""
    if speed_mps < 6.0 or vla_deg < 2.0:
        return 0.0                                   # putts / topped shots: rolling, not modelled
    ds = (speed_mps - _TABLE[:, 0]) / (0.05 * _TABLE[:, 0])
    da = (vla_deg - _TABLE[:, 1]) / 1.5
    d2 = ds * ds + da * da
    w = np.exp(-0.5 * (d2 - d2.min()))
    # spin scales roughly with speed for the same delivered loft
    est = float(np.sum(w * _TABLE[:, 2] * np.clip(speed_mps / _TABLE[:, 0], 0.2, 1.3)) / np.sum(w))
    return float(min(12000.0, max(300.0, est)))


def plausible_spin_limit_rpm(speed_mps: float, vla_deg: float) -> float:
    """Upper bound of the spin search window: generous, but it keeps the search
    out of regions that no golf shot reaches (limits aliasing on symmetric
    dimple patterns)."""
    return float(min(13000.0, max(5000.0, 2.2 * empirical_spin_rpm(speed_mps, vla_deg))))
