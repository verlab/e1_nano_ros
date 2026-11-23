"""A small NatNet server that streams rigid bodies like Motive does (for tests and simulation).

It answers NAT_CONNECT and NAT_REQUEST_MODELDEF and sends NAT_FRAMEOFDATA either to a multicast
group or, in unicast mode, to every client that connected (at its IP, ``data_port``). Only the
rigid body part of a frame is filled; that is all ``NatNetClient`` and most robotics clients read.

    srv = NatNetServer(lambda: [RigidBody(1, (0, 0, 0.5), (1, 0, 0, 0))], names={1: 'e1_1'})
    srv.start(rate_hz=120)
"""
from __future__ import annotations

import socket
import threading
import time

from . import natnet as N


class NatNetServer:
    def __init__(self, source, names=None, ip='127.0.0.1', multicast=False, multicast_group=N.DEFAULT_MULTICAST,
                 command_port=N.COMMAND_PORT, data_port=N.DATA_PORT, version: N.Version = N.Version(4, 1)):
        self.source, self.names, self.version = source, dict(names or {}), version
        self.ip, self.multicast, self.group = ip, multicast, multicast_group
        self.command_port, self.data_port = command_port, data_port
        self.clients = set()
        self.frame = 0
        self._stop = threading.Event()
        self._threads = []
        self.cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.cmd.bind((ip, command_port))
        self.cmd.settimeout(0.2)
        self.data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        if multicast:
            self.data.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
            self.data.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(ip))

    def start(self, rate_hz=120.0):
        for fn, args in ((self._cmd_loop, ()), (self._stream, (rate_hz,))):
            th = threading.Thread(target=fn, args=args, daemon=True)
            th.start()
            self._threads.append(th)
        return self

    def stop(self):
        self._stop.set()
        for th in self._threads:
            th.join(1.0)
        self.cmd.close()
        self.data.close()

    def _cmd_loop(self):
        while not self._stop.is_set():
            try:
                buf, addr = self.cmd.recvfrom(65536)
            except (socket.timeout, OSError):
                continue
            if len(buf) < 4:
                continue
            msg, _ = N._HDR.unpack_from(buf)
            if msg == N.NAT_CONNECT:
                self.clients.add(addr[0])
                self.cmd.sendto(N.packet(N.NAT_SERVERINFO, N.encode_server_info(version=self.version)), addr)
            elif msg == N.NAT_REQUEST_MODELDEF:
                self.cmd.sendto(N.packet(N.NAT_MODELDEF, N.encode_model_def(self.names, self.version)), addr)
            elif msg == N.NAT_KEEPALIVE:
                self.clients.add(addr[0])
            elif msg == N.NAT_DISCONNECT:
                self.clients.discard(addr[0])

    def send_frame(self, bodies):
        pkt = N.packet(N.NAT_FRAMEOFDATA, N.encode_frame(self.frame, bodies, self.version))
        self.frame += 1
        dests = [self.group] if self.multicast else list(self.clients)
        for ip in dests:
            try:
                self.data.sendto(pkt, (ip, self.data_port))
            except OSError:
                pass

    def _stream(self, rate_hz):
        period, nxt = 1.0 / rate_hz, time.monotonic()
        while not self._stop.is_set():
            self.send_frame(self.source())
            nxt += period
            self._stop.wait(max(0.0, nxt - time.monotonic()))
