"""Ground-side position control for the E1 Nano (runs on a PC, one instance per drone).

The E1 Nano has no position sensor: position comes from motion capture (or the simulator)
through ``e1/<id>/pose``. ``PositionController`` turns a position/velocity/acceleration
setpoint into the firmware-native attitude command (quaternion + collective duty), sent on
``e1/<id>/cmd/attitude`` at 50-100 Hz. This is the same split used by Crazyswarm-style
systems and scales to many drones because every vehicle is closed through one cheap message.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .math3d import quat_from_thrust_dir

G = 9.81


@dataclass
class VehicleModel:
    mass: float = 0.0251            # kg (E1 Nano with 1S 250 mAh)
    max_thrust: float = 0.216       # N per motor at duty = 1 (full battery)
    hover_duty: float = math.nan    # optional measured hover duty (overrides the model)

    def duty_for(self, force_n, battery_v=4.2):
        """Collective duty for a total thrust in newtons (thrust ~ duty^2, ~ V^2)."""
        tmax = self.max_thrust * (max(battery_v, 3.0) / 4.2) ** 2
        return math.sqrt(max(force_n, 0.0) / (4 * tmax))


@dataclass
class Gains:
    kp: np.ndarray = field(default_factory=lambda: np.array([4.0, 4.0, 5.0]))
    kd: np.ndarray = field(default_factory=lambda: np.array([3.0, 3.0, 3.5]))
    ki: np.ndarray = field(default_factory=lambda: np.array([0.4, 0.4, 1.2]))
    i_lim: float = 1.5               # m/s^2
    tilt_max: float = math.radians(25)
    acc_max: float = 6.0


class PositionController:
    def __init__(self, model: VehicleModel | None = None, gains: Gains | None = None):
        self.m = model or VehicleModel()
        self.g = gains or Gains()
        self.integral = np.zeros(3)
        self._vel_est = None
        self._last = None

    def reset(self):
        self.integral[:] = 0
        self._vel_est = None
        self._last = None

    def velocity(self, pos, t, vel=None):
        """Use measured velocity if given, else a filtered finite difference of the pose."""
        if vel is not None and not np.any(np.isnan(vel)):
            return np.asarray(vel, float)
        if self._last is None:
            self._last = (t, np.array(pos, float))
            self._vel_est = np.zeros(3)
            return self._vel_est
        t0, p0 = self._last
        dt = t - t0
        if dt > 1e-4:
            v = (np.asarray(pos) - p0) / dt
            self._vel_est = 0.6 * self._vel_est + 0.4 * v
            self._last = (t, np.array(pos, float))
        return self._vel_est

    def update(self, t, pos, vel, yaw, sp_pos, sp_vel=None, sp_acc=None, sp_yaw=0.0, battery_v=4.2, dt=0.02, extra_acc=None):
        """Returns (quat w x y z, duty) for CmdAttitude."""
        pos = np.asarray(pos, float)
        vel = self.velocity(pos, t, vel)
        sp_vel = np.zeros(3) if sp_vel is None else np.asarray(sp_vel, float)
        sp_acc = np.zeros(3) if sp_acc is None else np.asarray(sp_acc, float)
        e = np.asarray(sp_pos, float) - pos
        self.integral = np.clip(self.integral + self.g.ki * e * dt, -self.g.i_lim, self.g.i_lim)
        acc = sp_acc + self.g.kp * e + self.g.kd * (sp_vel - vel) + self.integral
        if extra_acc is not None:
            acc = acc + extra_acc
        n = np.linalg.norm(acc[:2])
        if n > self.g.acc_max:
            acc[:2] *= self.g.acc_max / n
        f = acc + np.array([0, 0, G])
        f[2] = max(f[2], 0.3 * G)
        horiz = np.linalg.norm(f[:2])
        max_h = f[2] * math.tan(self.g.tilt_max)
        if horiz > max_h:
            f[:2] *= max_h / horiz
        thrust_n = self.m.mass * np.linalg.norm(f)
        q = quat_from_thrust_dir(f / np.linalg.norm(f), sp_yaw)
        duty = self.m.duty_for(thrust_n, battery_v)
        return q, float(np.clip(duty, 0.12, 0.95))


# ---------------------------------------------------------------------------- trajectories
def min_jerk(p0, p1, T, t):
    """Position, velocity, acceleration of a min-jerk segment p0->p1 of duration T at time t."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    s = min(max(t / T, 0.0), 1.0) if T > 0 else 1.0
    sp = 10 * s**3 - 15 * s**4 + 6 * s**5
    sv = (30 * s**2 - 60 * s**3 + 30 * s**4) / T if T > 0 else 0.0
    sa = (60 * s - 180 * s**2 + 120 * s**3) / T**2 if T > 0 else 0.0
    return p0 + (p1 - p0) * sp, (p1 - p0) * sv, (p1 - p0) * sa


class Waypoints:
    """Piecewise min-jerk trajectory through (point, duration) pairs."""

    def __init__(self, start, legs):
        self.pts = [np.asarray(start, float)] + [np.asarray(p, float) for p, _ in legs]
        self.durs = [d for _, d in legs]
        self.t_end = float(sum(self.durs))

    def __call__(self, t):
        acc_t = 0.0
        for k, d in enumerate(self.durs):
            if t <= acc_t + d:
                return min_jerk(self.pts[k], self.pts[k + 1], d, t - acc_t)
            acc_t += d
        return self.pts[-1], np.zeros(3), np.zeros(3)


def square(center, side, z, leg_time=2.5, hold=0.5):
    cx, cy = center
    h = side / 2
    corners = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h), (cx - h, cy - h)]
    legs = []
    for x, y in corners:
        legs += [((x, y, z), leg_time), ((x, y, z), hold)]
    return legs


def circle_formation(n, radius, center=(0.0, 0.0), z=0.5, phase=0.0):
    """n positions evenly spread on a circle."""
    return [np.array([center[0] + radius * math.cos(phase + 2 * math.pi * k / n),
                      center[1] + radius * math.sin(phase + 2 * math.pi * k / n), z]) for k in range(n)]


def grid_formation(n, spacing=0.4, z=0.5, cols=None):
    cols = cols or int(math.ceil(math.sqrt(n)))
    rows = int(math.ceil(n / cols))
    out = []
    for k in range(n):
        r, c = divmod(k, cols)
        out.append(np.array([(c - (cols - 1) / 2) * spacing, (r - (rows - 1) / 2) * spacing, z]))
    return out


def separation_acc(i, positions: dict, d_min=0.25, gain=3.0, z_scale=0.5):
    """Repulsive acceleration (m/s^2) that keeps drone i at least d_min from the others.

    Vertical distances are scaled by z_scale (downwash makes stacking riskier than side-by-side)."""
    if d_min <= 0 or i not in positions:
        return np.zeros(3)
    pi = np.asarray(positions[i], float)
    acc = np.zeros(3)
    for j, pj in positions.items():
        if j == i or pj is None:
            continue
        d = pi - np.asarray(pj, float)
        d[2] /= z_scale
        n = np.linalg.norm(d)
        if 1e-6 < n < d_min:
            acc += gain * (d_min - n) / d_min * d / n * 9.81 * 0.3
    acc[2] *= z_scale
    return acc
