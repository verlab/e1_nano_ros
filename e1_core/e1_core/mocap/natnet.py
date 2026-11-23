"""Minimal OptiTrack NatNet client (rigid bodies only), pure Python, no SDK needed.

Talks to Motive over UDP: commands on port 1510, frames on port 1511 (multicast
``239.255.42.99`` or unicast). Only what a drone swarm needs is decoded: rigid body id,
position, orientation and tracking-valid flag, plus rigid body names from the model
definitions. Supports NatNet 3.x and 4.x streams (Motive 2.x/3.x).

    c = NatNetClient(server_ip='192.168.1.10', local_ip='192.168.1.20', multicast=True)
    c.on_frame = lambda frame, bodies, t: print(frame, bodies)
    c.start(); print(c.names)            # {streaming id: rigid body name}

Layout follows the reference ``NatNetClient.py`` shipped with the NatNet SDK 4.1.
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass, field

NAT_CONNECT = 0
NAT_SERVERINFO = 1
NAT_REQUEST = 2
NAT_RESPONSE = 3
NAT_REQUEST_MODELDEF = 4
NAT_MODELDEF = 5
NAT_REQUEST_FRAMEOFDATA = 6
NAT_FRAMEOFDATA = 7
NAT_MESSAGESTRING = 8
NAT_DISCONNECT = 9
NAT_KEEPALIVE = 10
NAT_UNRECOGNIZED_REQUEST = 100

DATASET_MARKERSET = 0
DATASET_RIGIDBODY = 1

DEFAULT_MULTICAST = '239.255.42.99'
COMMAND_PORT = 1510
DATA_PORT = 1511

_HDR = struct.Struct('<HH')
_I32 = struct.Struct('<i')
_RB = struct.Struct('<i3f4f')       # id, x y z, qx qy qz qw
_RB_TAIL = struct.Struct('<fh')     # mean marker error, params (bit 0 = tracking valid)


@dataclass
class RigidBody:
    id: int
    pos: tuple                    # m, Motive frame
    quat: tuple                   # (w, x, y, z), Motive frame
    valid: bool = True
    error: float = 0.0            # mean marker error, m
    name: str = ''


@dataclass
class Version:
    major: int = 4
    minor: int = 1

    @property
    def sized(self):
        """NatNet >= 4.1 prefixes every data block with its size in bytes."""
        return self.major > 4 or (self.major == 4 and self.minor > 0)


def _cstr(buf, off):
    end = buf.index(b'\0', off)
    return buf[off:end].decode(errors='replace'), end + 1


# ----------------------------------------------------------------------------- decoding
def decode_frame(payload: bytes, version: Version = Version()):
    """Decode a NAT_FRAMEOFDATA payload up to the rigid bodies. Returns (frame_number, [RigidBody])."""
    off = 0
    frame, = _I32.unpack_from(payload, off); off += 4
    # marker sets
    n, = _I32.unpack_from(payload, off); off += 4
    if version.sized:
        size, = _I32.unpack_from(payload, off); off += 4
        off += size
    else:
        for _ in range(n):
            _, off = _cstr(payload, off)
            m, = _I32.unpack_from(payload, off); off += 4 + 12 * m
    # legacy unlabeled markers
    n, = _I32.unpack_from(payload, off); off += 4
    if version.sized:
        size, = _I32.unpack_from(payload, off); off += 4
        off += size
    else:
        off += 12 * n
    # rigid bodies
    n, = _I32.unpack_from(payload, off); off += 4
    if version.sized:
        off += 4
    bodies = []
    for _ in range(n):
        rid, x, y, z, qx, qy, qz, qw = _RB.unpack_from(payload, off); off += _RB.size
        if 0 < version.major < 3:
            m, = _I32.unpack_from(payload, off); off += 4 + 12 * m
            if version.major >= 2:
                off += 8 * m          # marker ids + sizes
        err, valid = 0.0, True
        if version.major >= 2 or version.major == 0:
            err, = struct.unpack_from('<f', payload, off); off += 4
        if version.major > 2 or version.major == 0 or (version.major == 2 and version.minor >= 6):
            params, = struct.unpack_from('<h', payload, off); off += 2
            valid = bool(params & 0x01)
        bodies.append(RigidBody(rid, (x, y, z), (qw, qx, qy, qz), valid, err))
    return frame, bodies


def decode_model_def(payload: bytes, version: Version = Version()):
    """Return {rigid body id: name} from a NAT_MODELDEF payload."""
    names, off = {}, 0
    n, = _I32.unpack_from(payload, off); off += 4
    for _ in range(n):
        kind, = _I32.unpack_from(payload, off); off += 4
        end = None
        if version.sized:
            size, = _I32.unpack_from(payload, off); off += 4
            end = off + size
        if kind == DATASET_RIGIDBODY:
            name, o = _cstr(payload, off) if version.major >= 2 or version.major == 0 else ('', off)
            rid, = _I32.unpack_from(payload, o)
            names[rid] = name
            if end is None:
                o += 4 + 4 + 12                       # id, parent, offset
                if version.major >= 3 or version.major == 0:
                    m, = _I32.unpack_from(payload, o); o += 4 + 12 * m + 4 * m
                    if version.major >= 4 or version.major == 0:
                        for _ in range(m):
                            _, o = _cstr(payload, o)
                off = o
        elif kind == DATASET_MARKERSET and end is None:
            _, off = _cstr(payload, off)
            m, = _I32.unpack_from(payload, off); off += 4
            for _ in range(m):
                _, off = _cstr(payload, off)
        elif end is None:
            break                                     # unknown dataset without size: stop
        if end is not None:
            off = end
    return names


# ----------------------------------------------------------------------------- encoding (tests, fake server)
def encode_frame(frame: int, bodies, version: Version = Version()) -> bytes:
    """NAT_FRAMEOFDATA payload with no marker sets and the given rigid bodies (NatNet >= 3)."""
    out = [_I32.pack(frame), _I32.pack(0)] + ([_I32.pack(0)] if version.sized else [])
    out += [_I32.pack(0)] + ([_I32.pack(0)] if version.sized else [])
    rbs = b''.join(_RB.pack(b.id, *b.pos, b.quat[1], b.quat[2], b.quat[3], b.quat[0]) +
                   _RB_TAIL.pack(b.error, 1 if b.valid else 0) for b in bodies)
    out += [_I32.pack(len(bodies))] + ([_I32.pack(len(rbs))] if version.sized else []) + [rbs]
    out += [_I32.pack(0)] * 6       # skeletons, labeled markers, force plates, devices (+ sizes): left empty
    return b''.join(out)


def encode_model_def(names: dict, version: Version = Version()) -> bytes:
    out = [_I32.pack(len(names))]
    for rid, name in names.items():
        body = name.encode() + b'\0' + struct.pack('<ii3f', rid, -1, 0, 0, 0) + _I32.pack(0)
        out += [_I32.pack(DATASET_RIGIDBODY)] + ([_I32.pack(len(body))] if version.sized else []) + [body]
    return b''.join(out)


def packet(msg_id: int, payload: bytes = b'') -> bytes:
    return _HDR.pack(msg_id, len(payload)) + payload


def decode_server_info(payload: bytes):
    name = payload[:256].split(b'\0', 1)[0].decode(errors='replace')
    app = tuple(payload[256:260])
    nn = tuple(payload[260:264])
    return name, app, Version(nn[0], nn[1])


def encode_server_info(name='E1 NatNet test server', version: Version = Version()) -> bytes:
    return name.encode().ljust(256, b'\0') + bytes([3, 1, 0, 0]) + bytes([version.major, version.minor, 0, 0])


# ----------------------------------------------------------------------------- client
@dataclass
class _Stats:
    frames: int = 0
    last_frame: int = -1
    t_last: float = 0.0
    errors: list = field(default_factory=list)


class NatNetClient:
    """Receive rigid body poses from Motive. Callback: ``on_frame(frame_number, bodies, t_monotonic)``."""

    def __init__(self, server_ip='127.0.0.1', local_ip='0.0.0.0', multicast=True, multicast_group=DEFAULT_MULTICAST,
                 command_port=COMMAND_PORT, data_port=DATA_PORT, version: Version | None = None):
        self.server = (server_ip, command_port)
        self.local_ip, self.multicast, self.group, self.data_port = local_ip, multicast, multicast_group, data_port
        self.version = version            # None = ask the server
        self.server_name = ''
        self.names: dict[int, str] = {}
        self.on_frame = None
        self.stats = _Stats()
        self._stop = threading.Event()
        self._threads = []
        self._cmd = self._data = None
        self._replies = {}
        self._reply_evt = threading.Event()

    # sockets ------------------------------------------------------------------
    def _open(self):
        self._cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._cmd.bind((self.local_ip, 0))
        self._cmd.settimeout(0.2)
        d = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        d.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            d.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        d.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        if self.multicast:
            d.bind(('', self.data_port))
            mreq = socket.inet_aton(self.group) + socket.inet_aton(self.local_ip)
            d.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        else:
            d.bind((self.local_ip, self.data_port))
        d.settimeout(0.2)
        self._data = d

    def send(self, msg_id, payload=b''):
        self._cmd.sendto(packet(msg_id, payload), self.server)

    def request(self, msg_id, payload=b'', reply=None, timeout=1.0):
        self._reply_evt.clear()
        self._replies.pop(reply, None)
        self.send(msg_id, payload)
        end = time.monotonic() + timeout
        while reply is not None and reply not in self._replies and time.monotonic() < end:
            self._reply_evt.wait(0.05)
        return self._replies.get(reply)

    # lifecycle ----------------------------------------------------------------
    def start(self, timeout=2.0):
        self._open()
        for fn in (self._cmd_loop, self._data_loop):
            th = threading.Thread(target=fn, daemon=True)
            th.start()
            self._threads.append(th)
        info = self.request(NAT_CONNECT, b'Ping\0', NAT_SERVERINFO, timeout)
        if info is not None:
            self.server_name, _, v = decode_server_info(info)
            self.version = self.version or v
        elif self.version is None:
            self.version = Version(4, 1)          # no answer: assume a recent Motive
        self.refresh_names(timeout)
        if not self.multicast:
            th = threading.Thread(target=self._keepalive, daemon=True)
            th.start()
            self._threads.append(th)
        return info is not None

    def refresh_names(self, timeout=1.0):
        md = self.request(NAT_REQUEST_MODELDEF, b'', NAT_MODELDEF, timeout)
        if md is not None:
            try:
                self.names = decode_model_def(md, self.version)
            except (struct.error, ValueError) as e:
                self.stats.errors.append(f'model def: {e}')
        return self.names

    def stop(self):
        self._stop.set()
        for th in self._threads:
            th.join(1.0)
        for s in (self._cmd, self._data):
            if s is not None:
                s.close()

    # loops --------------------------------------------------------------------
    def _keepalive(self):
        while not self._stop.wait(1.0):
            try:
                self.send(NAT_KEEPALIVE)
            except OSError:
                pass

    def _cmd_loop(self):
        while not self._stop.is_set():
            try:
                buf, _ = self._cmd.recvfrom(65536)
            except (socket.timeout, OSError):
                continue
            if len(buf) < 4:
                continue
            msg, n = _HDR.unpack_from(buf)
            if msg == NAT_FRAMEOFDATA:            # unicast servers may answer on the command socket
                self._handle_frame(buf[4:4 + n])
            else:
                self._replies[msg] = buf[4:4 + n]
                self._reply_evt.set()

    def _data_loop(self):
        while not self._stop.is_set():
            try:
                buf, _ = self._data.recvfrom(65536)
            except (socket.timeout, OSError):
                continue
            if len(buf) >= 4:
                msg, n = _HDR.unpack_from(buf)
                if msg == NAT_FRAMEOFDATA:
                    self._handle_frame(buf[4:4 + n])
                elif msg == NAT_MODELDEF:
                    self._replies[msg] = buf[4:4 + n]
                    self._reply_evt.set()

    def _handle_frame(self, payload):
        t = time.monotonic()
        try:
            frame, bodies = decode_frame(payload, self.version or Version())
        except (struct.error, ValueError) as e:
            self.stats.errors.append(str(e))
            return
        for b in bodies:
            b.name = self.names.get(b.id, '')
        self.stats.frames += 1
        self.stats.last_frame, self.stats.t_last = frame, t
        if self.on_frame:
            self.on_frame(frame, bodies, t)
