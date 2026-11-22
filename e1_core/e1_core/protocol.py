"""E1 wire protocol over Zenoh (version 1).

Every drone owns the key space ``e1/<id>/**`` (``<id>`` is a small integer, 1..254).
Payloads are fixed-size little-endian structs so they can be produced/consumed by the
ESP32 firmware (``firmware/src_e1/e1_protocol.h``) with zero allocation.

Key expressions
---------------
=========================  ==========  ===========================================
key                        direction   payload
=========================  ==========  ===========================================
e1/<id>/alive              drone→      Zenoh liveliness token (discovery)
e1/<id>/state              drone→      STATE     (50 Hz)
e1/<id>/log                drone→      UTF-8 text (console output)
e1/<id>/pose               →drone      POSE      (mocap / simulator ground truth)
e1/<id>/cmd/attitude       →drone      CMD_ATTITUDE (switches the drone to AUTO)
e1/<id>/cmd/manual         →drone      CMD_MANUAL   (stick emulation)
e1/<id>/cmd/motors         →drone      CMD_MOTORS   (direct duty cycles, AUTO)
e1/<id>/cmd/arm            →drone      CMD_ARM
e1/<id>/cmd/mode           →drone      CMD_MODE
e1/<id>/cmd/param          →drone      CMD_PARAM   (set firmware parameter)
e1/<id>/cmd/console        →drone      UTF-8 CLI command (same as serial console)
e1/all/cmd/<x>             →all        broadcast of any cmd (e.g. emergency disarm)
=========================  ==========  ===========================================

Frames: body FLU (x forward, y left, z up). Attitude quaternions are (w, x, y, z) and rotate
body→world (world = ENU-like, z up). Thrust is the firmware-native normalized collective
duty cycle in [0, 1] (≈0.5 hovers the E1 Nano on a full battery).
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Tuple

VERSION = 1
PREFIX = 'e1'
BROADCAST = 'all'

# message types
T_STATE, T_POSE, T_CMD_ATTITUDE, T_CMD_MANUAL, T_CMD_MOTORS, T_CMD_ARM, T_CMD_MODE, T_CMD_PARAM = range(1, 9)

# flight modes (same numbering as CF-Drone control.ino)
RAW, ACRO, STAB, ALTHOLD, AUTO = range(5)
MODE_NAMES = {RAW: 'RAW', ACRO: 'ACRO', STAB: 'STAB', ALTHOLD: 'ALTHOLD', AUTO: 'AUTO'}

# STATE.flags bits
F_LANDED, F_LOW_BATTERY, F_CMD_TIMEOUT, F_FAILSAFE, F_IMU_ERROR, F_POSE_VALID = (1 << i for i in range(6))

# attitude command type_mask bits (MAVLink-like)
IGNORE_RATES, IGNORE_ATTITUDE = 1, 2

_HDR = struct.Struct('<BBHI')            # version, type, seq, t_ms


def key(drone_id, *parts) -> str:
    return '/'.join([PREFIX, str(drone_id), *parts])


def parse_id(key_expr: str):
    """'e1/7/state' -> 7 ; 'e1/all/cmd/arm' -> 'all'."""
    p = str(key_expr).split('/')
    if len(p) < 2 or p[0] != PREFIX:
        return None
    return int(p[1]) if p[1].isdigit() else p[1]


@dataclass
class Header:
    type: int
    seq: int = 0
    t_ms: int = 0
    version: int = VERSION


def _pack(fmt: struct.Struct, mtype: int, seq: int, t_ms: int, *vals) -> bytes:
    return _HDR.pack(VERSION, mtype, seq & 0xFFFF, t_ms & 0xFFFFFFFF) + fmt.pack(*vals)


def _unpack(fmt: struct.Struct, mtype: int, buf: bytes):
    buf = bytes(buf)
    ver, t, seq, t_ms = _HDR.unpack_from(buf, 0)
    if ver != VERSION:
        raise ValueError(f'protocol version {ver} != {VERSION}')
    if t != mtype:
        raise ValueError(f'message type {t} != {mtype}')
    if len(buf) < _HDR.size + fmt.size:
        raise ValueError('short message')
    return Header(t, seq, t_ms, ver), fmt.unpack_from(buf, _HDR.size)


# --------------------------------------------------------------------------- STATE
_STATE = struct.Struct('<4f3f3f4fffBBH')


@dataclass
class State:
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)   # body->world, w x y z
    rates: Tuple[float, float, float] = (0.0, 0.0, 0.0)              # rad/s, body FLU
    acc: Tuple[float, float, float] = (0.0, 0.0, 9.81)               # m/s^2, body FLU (specific force)
    motors: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)  # duty, order RL RR FR FL
    thrust: float = 0.0                                               # collective thrust target (duty)
    battery_v: float = 4.2
    mode: int = STAB
    armed: bool = False
    flags: int = 0
    seq: int = 0
    t_ms: int = 0

    def encode(self) -> bytes:
        return _pack(_STATE, T_STATE, self.seq, self.t_ms, *self.quat, *self.rates, *self.acc, *self.motors,
                     self.thrust, self.battery_v, self.mode, int(self.armed), self.flags)

    @classmethod
    def decode(cls, buf) -> 'State':
        h, v = _unpack(_STATE, T_STATE, buf)
        return cls(tuple(v[0:4]), tuple(v[4:7]), tuple(v[7:10]), tuple(v[10:14]), v[14], v[15], v[16],
                   bool(v[17]), v[18], h.seq, h.t_ms)


# --------------------------------------------------------------------------- POSE
_POSE = struct.Struct('<3f4f3f')


@dataclass
class Pose:
    pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)               # m, world
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)   # body->world
    vel: Tuple[float, float, float] = (math.nan, math.nan, math.nan)  # m/s world (NaN = unknown)
    seq: int = 0
    t_ms: int = 0

    def encode(self) -> bytes:
        return _pack(_POSE, T_POSE, self.seq, self.t_ms, *self.pos, *self.quat, *self.vel)

    @classmethod
    def decode(cls, buf) -> 'Pose':
        h, v = _unpack(_POSE, T_POSE, buf)
        return cls(tuple(v[0:3]), tuple(v[3:7]), tuple(v[7:10]), h.seq, h.t_ms)


# --------------------------------------------------------------------------- commands
_ATT = struct.Struct('<4f3ffB')


@dataclass
class CmdAttitude:
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    rates: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # feed-forward body rates, rad/s
    thrust: float = 0.0                                   # collective duty [0,1]; >0 also arms (AUTO)
    type_mask: int = IGNORE_RATES
    seq: int = 0
    t_ms: int = 0

    def encode(self) -> bytes:
        return _pack(_ATT, T_CMD_ATTITUDE, self.seq, self.t_ms, *self.quat, *self.rates, self.thrust, self.type_mask)

    @classmethod
    def decode(cls, buf) -> 'CmdAttitude':
        h, v = _unpack(_ATT, T_CMD_ATTITUDE, buf)
        return cls(tuple(v[0:4]), tuple(v[4:7]), v[7], v[8], h.seq, h.t_ms)


_MAN = struct.Struct('<5f')


@dataclass
class CmdManual:
    roll: float = 0.0       # [-1, 1]
    pitch: float = 0.0      # [-1, 1]
    yaw: float = 0.0        # [-1, 1] (positive = clockwise, like the RC stick)
    throttle: float = 0.0   # [0, 1]
    mode: float = math.nan  # optional RC mode switch value [0, 1]
    seq: int = 0
    t_ms: int = 0

    def encode(self) -> bytes:
        return _pack(_MAN, T_CMD_MANUAL, self.seq, self.t_ms, self.roll, self.pitch, self.yaw, self.throttle, self.mode)

    @classmethod
    def decode(cls, buf) -> 'CmdManual':
        h, v = _unpack(_MAN, T_CMD_MANUAL, buf)
        return cls(*v, seq=h.seq, t_ms=h.t_ms)


_MOT = struct.Struct('<4f')


@dataclass
class CmdMotors:
    duty: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)   # RL RR FR FL
    seq: int = 0
    t_ms: int = 0

    def encode(self) -> bytes:
        return _pack(_MOT, T_CMD_MOTORS, self.seq, self.t_ms, *self.duty)

    @classmethod
    def decode(cls, buf) -> 'CmdMotors':
        h, v = _unpack(_MOT, T_CMD_MOTORS, buf)
        return cls(tuple(v), h.seq, h.t_ms)


_ARM = struct.Struct('<B')


@dataclass
class CmdArm:
    arm: bool = False
    seq: int = 0
    t_ms: int = 0

    def encode(self) -> bytes:
        return _pack(_ARM, T_CMD_ARM, self.seq, self.t_ms, int(self.arm))

    @classmethod
    def decode(cls, buf) -> 'CmdArm':
        h, v = _unpack(_ARM, T_CMD_ARM, buf)
        return cls(bool(v[0]), h.seq, h.t_ms)


_MODE = struct.Struct('<B')


@dataclass
class CmdMode:
    mode: int = STAB
    seq: int = 0
    t_ms: int = 0

    def encode(self) -> bytes:
        return _pack(_MODE, T_CMD_MODE, self.seq, self.t_ms, self.mode)

    @classmethod
    def decode(cls, buf) -> 'CmdMode':
        h, v = _unpack(_MODE, T_CMD_MODE, buf)
        return cls(v[0], h.seq, h.t_ms)


_PARAM = struct.Struct('<16sf')


@dataclass
class CmdParam:
    name: str = ''
    value: float = 0.0
    seq: int = 0
    t_ms: int = 0

    def encode(self) -> bytes:
        return _pack(_PARAM, T_CMD_PARAM, self.seq, self.t_ms, self.name.encode()[:16], self.value)

    @classmethod
    def decode(cls, buf) -> 'CmdParam':
        h, v = _unpack(_PARAM, T_CMD_PARAM, buf)
        return cls(v[0].split(b'\0', 1)[0].decode(), v[1], h.seq, h.t_ms)


SIZES = {'STATE': _HDR.size + _STATE.size, 'POSE': _HDR.size + _POSE.size, 'CMD_ATTITUDE': _HDR.size + _ATT.size,
         'CMD_MANUAL': _HDR.size + _MAN.size, 'CMD_MOTORS': _HDR.size + _MOT.size, 'CMD_ARM': _HDR.size + _ARM.size,
         'CMD_MODE': _HDR.size + _MODE.size, 'CMD_PARAM': _HDR.size + _PARAM.size}

DECODERS = {'state': State.decode, 'pose': Pose.decode, 'cmd/attitude': CmdAttitude.decode,
            'cmd/manual': CmdManual.decode, 'cmd/motors': CmdMotors.decode, 'cmd/arm': CmdArm.decode,
            'cmd/mode': CmdMode.decode, 'cmd/param': CmdParam.decode}
