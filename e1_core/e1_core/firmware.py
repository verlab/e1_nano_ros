"""Python port of the CF-Drone flight controller (control.ino / pid.h / safety.ino / motors.ino).

Used by the simulator as "firmware in the loop": every simulated E1 Nano runs this exact
controller and speaks the same Zenoh protocol as the real ESP32 (see ``protocol.py``), so
ground software cannot tell the difference between simulation and hardware.

Gains, limits, the mixer, flight modes, arming rules and the descend failsafe follow
CF-Drone commit 83ebe61 (github.com/songge8/CF-Drone).
"""
from __future__ import annotations

import math
import numpy as np

from . import protocol as P
from .math3d import from_euler, qrot_inv, rotation_vector_between, wrap, yaw_of

RAW, ACRO, STAB, ALTHOLD, AUTO = P.RAW, P.ACRO, P.STAB, P.ALTHOLD, P.AUTO
RL, RR, FR, FL = 0, 1, 2, 3          # CF-Drone motor indexes


class LowPass:
    def __init__(self, alpha):
        self.alpha, self.out = alpha, None

    def update(self, x):
        if self.alpha == 1:
            return x
        self.out = x if self.out is None else self.out + self.alpha * (x - self.out)
        return self.out

    def reset(self):
        self.out = None


class PID:
    def __init__(self, p, i, d, windup=0.0, d_alpha=1.0, dt_max=0.1):
        self.p, self.i, self.d, self.windup, self.dt_max = p, i, d, windup, dt_max
        self.lpf = LowPass(d_alpha)
        self.reset()

    def reset(self):
        self.prev_err = math.nan
        self.prev_t = math.nan
        self.integral = 0.0
        self.derivative = 0.0
        self.lpf.reset()

    def update(self, err, t):
        dt = t - self.prev_t
        if dt > 0 and dt < self.dt_max:
            self.integral += err * dt
            self.derivative = self.lpf.update((err - self.prev_err) / dt)
        else:
            self.integral = 0.0
            self.derivative = 0.0
        self.prev_err, self.prev_t = err, t
        return self.p * err + max(-self.windup, min(self.windup, self.i * self.integral)) + self.d * self.derivative


class Params(dict):
    """Firmware parameters (subset of CF-Drone parameters.ino), settable through CMD_PARAM."""
    DEFAULTS = {
        'CTL_R_RATE_P': 0.06, 'CTL_R_RATE_I': 0.1, 'CTL_R_RATE_D': 0.001,
        'CTL_P_RATE_P': 0.06, 'CTL_P_RATE_I': 0.1, 'CTL_P_RATE_D': 0.001,
        'CTL_Y_RATE_P': 0.3, 'CTL_Y_RATE_I': 0.01, 'CTL_Y_RATE_D': 0.01,
        'CTL_R_P': 6.0, 'CTL_P_P': 6.0, 'CTL_Y_P': 3.0,
        'CTL_TILT_MAX': math.radians(30), 'MOT_THR_MIN': 0.10, 'MOT_THR_MAX': 0.9,
        'CTL_TRIM_ROLL': 0.0, 'CTL_TRIM_PITCH': 0.0,
        'ZN_CMD_TIMEOUT': 0.5,          # s without AUTO commands -> descend (E1 Zenoh link)
        'SF_DESCEND_TIME': 3.0,         # s to ramp thrust to zero in descend()
        'VBAT_WARN': 3.5, 'VBAT_CRIT': 3.3,
    }

    def __init__(self, **kw):
        super().__init__(self.DEFAULTS)
        self.update(kw)


class Firmware:
    """One E1 Nano flight controller instance."""

    def __init__(self, drone_id=1, **params):
        self.id = drone_id
        self.p = Params(**params)
        self._build_pids()
        self.mode = STAB
        self.armed = False
        self.t = 0.0
        self.attitude = np.array([1.0, 0, 0, 0])
        self.rates = np.zeros(3)
        self.acc = np.array([0, 0, 9.81])
        self.attitude_target = None           # quaternion or None (invalid)
        self.rates_target = None              # vector or None
        self.rates_extra = np.zeros(3)
        self.torque_target = np.zeros(3)
        self.thrust_target = 0.0
        self.motors = np.zeros(4)
        self.control = dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, mode=math.nan)
        self.manual_active = False
        self.last_auto_cmd = -1e9
        self.battery_v = 4.2
        self.flags = 0
        self.direct_motors = None
        self.log = []

    def _build_pids(self):
        p = self.p
        self.roll_rate = PID(p['CTL_R_RATE_P'], p['CTL_R_RATE_I'], p['CTL_R_RATE_D'], 0.3, 0.2)
        self.pitch_rate = PID(p['CTL_P_RATE_P'], p['CTL_P_RATE_I'], p['CTL_P_RATE_D'], 0.3, 0.2)
        self.yaw_rate = PID(p['CTL_Y_RATE_P'], p['CTL_Y_RATE_I'], p['CTL_Y_RATE_D'])
        self.roll_pid = PID(p['CTL_R_P'], 0, 0, math.radians(5))
        self.pitch_pid = PID(p['CTL_P_P'], 0, 0, math.radians(5))
        self.yaw_pid = PID(p['CTL_Y_P'], 0, 0)
        self.max_rate = np.radians([360, 360, 300])

    def print(self, s):
        self.log.append(s)

    # ------------------------------------------------------------------ command handlers (Zenoh link)
    def handle(self, kind, msg):
        if kind == 'cmd/attitude':
            if self.mode != AUTO:
                self.mode = AUTO
                for pid in (self.roll_rate, self.pitch_rate, self.yaw_rate, self.roll_pid, self.pitch_pid, self.yaw_pid):
                    pid.reset()
            self.direct_motors = None
            self.attitude_target = None if msg.type_mask & P.IGNORE_ATTITUDE else np.array(msg.quat, float)
            self.rates_target = None if msg.type_mask & P.IGNORE_ATTITUDE and msg.type_mask & P.IGNORE_RATES else np.array(msg.rates, float)
            self.rates_extra = np.zeros(3) if msg.type_mask & P.IGNORE_RATES else np.array(msg.rates, float)
            self.thrust_target = float(np.clip(msg.thrust, 0, 1))
            if msg.thrust > 0 and not self.armed:
                self._try_arm()
            elif msg.thrust <= 0:
                self.armed = False
            self.last_auto_cmd = self.t
        elif kind == 'cmd/manual':
            self.control.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, throttle=msg.throttle, mode=msg.mode)
            self.manual_active = True
            if self.mode == AUTO:          # autoFailsafe(): pilot input takes over
                self.mode = STAB
        elif kind == 'cmd/motors':
            self.mode = AUTO
            self.attitude_target = self.rates_target = None
            self.direct_motors = np.clip(np.array(msg.duty, float), 0, 1)
            self.armed = bool(np.any(self.direct_motors > 0))
            self.last_auto_cmd = self.t
        elif kind == 'cmd/arm':
            if msg.arm:
                self._try_arm()
            else:
                self.armed = False
        elif kind == 'cmd/mode':
            self.mode = int(msg.mode)
            self.last_auto_cmd = self.t
        elif kind == 'cmd/param':
            if msg.name in self.p:
                self.p[msg.name] = msg.value
                self._build_pids()
                self.print(f'{msg.name} = {msg.value:g}')
            else:
                self.print(f'unknown parameter {msg.name}')

    def _try_arm(self):
        if self.battery_v < self.p['VBAT_WARN'] and self.battery_v > 1.0:
            self.print(f'battery low ({self.battery_v:.2f} V), arming refused')
            return
        self.armed = True

    # ------------------------------------------------------------------ main loop step
    def step(self, t, attitude, gyro, acc, battery_v):
        self.dt = max(t - self.t, 0.0)
        self.t, self.attitude, self.rates, self.acc, self.battery_v = t, np.asarray(attitude), np.asarray(gyro), np.asarray(acc), battery_v
        self.flags = 0
        if self.manual_active and self.mode != AUTO:
            self._interpret_controls()
        self._failsafe()
        if self.direct_motors is not None and self.mode == AUTO:
            self.motors = self.direct_motors if self.armed else np.zeros(4)
            return self.motors
        self._control_attitude()
        self._control_rates()
        self._control_torque()
        return self.motors

    def _interpret_controls(self):
        c = self.control
        if not math.isnan(c['mode']):
            pass  # single-mode setup (flightModes = STAB, STAB, STAB)
        if c['throttle'] < 0.05 and c['yaw'] > 0.95:
            self._try_arm()
        if c['throttle'] < 0.05 and c['yaw'] < -0.95:
            self.armed = False
        yaw_cmd = 0.0 if abs(c['yaw']) < 0.1 else c['yaw']
        thr = c['throttle']
        self.thrust_target = 0.0 if thr < 0.05 else np.interp(thr, [0.05, 1.0], [self.p['MOT_THR_MIN'], self.p['MOT_THR_MAX']])
        if self.mode == STAB:
            yt = yaw_of(self.attitude_target) if self.attitude_target is not None else math.nan
            if not self.armed or math.isnan(yt) or yaw_cmd != 0:
                yt = yaw_of(self.attitude)
            tm = self.p['CTL_TILT_MAX']
            self.attitude_target = from_euler(c['roll'] * tm + self.p['CTL_TRIM_ROLL'], c['pitch'] * tm + self.p['CTL_TRIM_PITCH'], yt)
            self.rates_extra = np.array([0, 0, -yaw_cmd * self.max_rate[2]])
        elif self.mode == ACRO:
            self.attitude_target = None
            self.rates_target = np.array([c['roll'], c['pitch'], -yaw_cmd]) * self.max_rate
        elif self.mode == RAW:
            self.attitude_target = self.rates_target = None
            self.torque_target = np.array([c['roll'], c['pitch'], -yaw_cmd]) * 0.1

    def _failsafe(self):
        if self.battery_v < self.p['VBAT_WARN']:
            self.flags |= P.F_LOW_BATTERY
        if self.mode == AUTO and self.armed and self.t - self.last_auto_cmd > self.p['ZN_CMD_TIMEOUT']:
            self.flags |= P.F_CMD_TIMEOUT | P.F_FAILSAFE
            self._descend()
        if self.armed and self.battery_v < self.p['VBAT_CRIT']:
            self.flags |= P.F_FAILSAFE
            self._descend()
        up_body = qrot_inv(self.attitude, [0, 0, 1])
        if up_body[2] < -0.5 and self.armed:              # invertedFailsafe (simplified, no delay)
            self.armed = False

    def _descend(self):
        yaw = yaw_of(self.attitude)
        self.direct_motors = None
        self.attitude_target = from_euler(0, 0, yaw)
        self.rates_extra = np.zeros(3)
        self.thrust_target = min(self.thrust_target, 0.5) - getattr(self, 'dt', 0.002) / self.p['SF_DESCEND_TIME']
        if self.thrust_target <= 0:
            self.thrust_target, self.armed = 0.0, False

    def _control_attitude(self):
        if not self.armed or self.attitude_target is None or self.thrust_target < self.p['MOT_THR_MIN']:
            return
        up = np.array([0, 0, 1.0])
        up_actual = qrot_inv(self.attitude, up)
        up_target = qrot_inv(self.attitude_target, up)
        err = rotation_vector_between(up_target, up_actual)
        rt = np.zeros(3)
        rt[0] = self.roll_pid.update(err[0], self.t) + self.rates_extra[0]
        rt[1] = self.pitch_pid.update(err[1], self.t) + self.rates_extra[1]
        rt[2] = self.yaw_pid.update(wrap(yaw_of(self.attitude_target) - yaw_of(self.attitude)), self.t) + self.rates_extra[2]
        self.rates_target = np.clip(rt, -self.max_rate, self.max_rate)

    def _control_rates(self):
        if not self.armed or self.rates_target is None or self.thrust_target < self.p['MOT_THR_MIN']:
            return
        e = self.rates_target - self.rates
        self.torque_target = np.array([self.roll_rate.update(e[0], self.t), self.pitch_rate.update(e[1], self.t),
                                       self.yaw_rate.update(e[2], self.t)])

    def _control_torque(self):
        if not self.armed:
            self.motors = np.zeros(4)
            return
        T = self.thrust_target
        if T < self.p['MOT_THR_MIN']:
            self.motors = np.full(4, self.p['MOT_THR_MIN'])
            return
        x, y, z = self.torque_target
        m = np.zeros(4)
        m[FL] = T + x - y + z
        m[FR] = T - x - y - z
        m[RL] = T + x + y - z
        m[RR] = T - x + y + z
        avg, mx = m.mean(), m.max()                       # desaturate (keep collective)
        if mx > 1.0 and mx > avg:
            m = avg + (m - avg) * (1.0 - avg) / (mx - avg)
        self.motors = np.clip(m, 0, 1)

    # ------------------------------------------------------------------ telemetry
    def state(self, seq=0):
        landed = (not self.armed)
        return P.State(tuple(self.attitude), tuple(self.rates), tuple(self.acc), tuple(self.motors),
                       float(self.thrust_target), float(self.battery_v), int(self.mode), bool(self.armed),
                       self.flags | (P.F_LANDED if landed else 0), seq, int(self.t * 1000))
