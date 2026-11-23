"""Frame conversions and pose post-processing for motion capture."""
from __future__ import annotations

import math

import numpy as np

from ..math3d import qconj, qmul

# Motive default is Y-up. A +90 deg rotation about x maps (x, y, z)_yup -> (x, -z, y)_zup.
_R_YUP_TO_ZUP = np.array([math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0])


def to_z_up(pos, quat_wxyz, up_axis='z'):
    """Express a mocap pose in a Z-up world frame. ``up_axis`` is Motive's streaming up axis."""
    p, q = np.asarray(pos, float), np.asarray(quat_wxyz, float)
    if up_axis.lower() == 'z':
        return p, q
    if up_axis.lower() != 'y':
        raise ValueError(f'unsupported up axis {up_axis!r}')
    p = np.array([p[0], -p[2], p[1]])
    q = qmul(qmul(_R_YUP_TO_ZUP, q), qconj(_R_YUP_TO_ZUP))
    return p, q / np.linalg.norm(q)


def from_z_up(pos, quat_wxyz, up_axis='z'):
    """Inverse of ``to_z_up``: express a Z-up pose in Motive's streaming frame."""
    p, q = np.asarray(pos, float), np.asarray(quat_wxyz, float)
    if up_axis.lower() == 'z':
        return tuple(p), tuple(q)
    p = np.array([p[0], p[2], -p[1]])
    q = qmul(qmul(qconj(_R_YUP_TO_ZUP), q), _R_YUP_TO_ZUP)
    return tuple(p), tuple(q / np.linalg.norm(q))


class PoseFilter:
    """Velocity by filtered finite differences, quaternion sign continuity and outlier rejection."""

    def __init__(self, alpha=0.35, max_jump=0.3, max_dt=0.2, max_rejects=5):
        self.alpha, self.max_jump, self.max_dt, self.max_rejects = alpha, max_jump, max_dt, max_rejects
        self.t = self.p = self.q = None
        self.v = np.zeros(3)
        self.rejects = 0              # consecutive rejected jumps; after max_rejects the filter resyncs

    def update(self, t, pos, quat):
        """Return (pos, quat, vel) or None if the sample is rejected as an outlier."""
        pos, quat = np.asarray(pos, float), np.asarray(quat, float)
        if self.q is not None and float(np.dot(self.q, quat)) < 0:
            quat = -quat
        if self.t is not None:
            dt = t - self.t
            if dt <= 0:
                return None
            if dt < self.max_dt and np.linalg.norm(pos - self.p) > self.max_jump and self.rejects < self.max_rejects:
                self.rejects += 1
                return None
            if dt < self.max_dt and self.rejects < self.max_rejects:
                self.v = (1 - self.alpha) * self.v + self.alpha * (pos - self.p) / dt
            else:
                self.v = np.zeros(3)          # gap or persistent jump: restart from this sample
        self.rejects = 0
        self.t, self.p, self.q = t, pos, quat
        return pos, quat, self.v.copy()
