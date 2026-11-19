#!/usr/bin/env python3
"""TEST-only CAR-LINK V1 wire simulation through an anonymous Linux PTY.

No configurable device path, no ROS node, no physical UART, no /cmd_vel.
0xE0-0xE2 are LOCAL TEST TYPES and must never be accepted by real STM32.
Response envelope uses the existing CAR-LINK V1 B2 feedback types 0x90-0x95.
"""
import argparse
import json
import os
import pty
import select
import struct
import time
import tty
from dataclasses import dataclass

from mission_execution import MissionError, SimExecutor, compile_workflow

MAGIC = b'\xA5\x5A'
VERSION = 1
MAX_PAYLOAD = 192
TEST_STEP = 0xE0
TEST_HELLO = 0xE1
TEST_QR = 0xE2
MOCK_MARKER = b'CAR_LINK_MOCK_ONLY_V1'
ACK, REJECTED, STARTED, PROGRESS, COMPLETED, FAILED = range(0x90, 0x96)
KINDS = ('MOVE_NODE', 'SCAN_QR', 'PICK_RAW', 'PLACE_ROUGH', 'PICK_ROUGH',
         'PLACE_STORE', 'STACK_STORE', 'HOME')
NODE_NAMES = ('', 'A1', 'A2', 'A3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3')


class WireError(MissionError):
    pass


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    msg_type: int
    payload: bytes
    sequence: int = 0
    timestamp_ms: int = 0
    flags: int = 0


def encode(frame):
    if len(frame.payload) > MAX_PAYLOAD:
        raise WireError('CAR-LINK payload too long')
    if not (0 <= frame.msg_type <= 255 and 0 <= frame.flags <= 255
            and 0 <= frame.sequence <= 65535 and 0 <= frame.timestamp_ms <= 0xFFFFFFFF):
        raise WireError('CAR-LINK field out of range')
    header = struct.pack('<2sBBBHHI', MAGIC, VERSION, frame.msg_type, frame.flags,
                         frame.sequence, len(frame.payload), frame.timestamp_ms)
    body = header + frame.payload
    return body + struct.pack('<H', crc16(body[2:]))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.length_errors = 0
        self.version_errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        if len(self.buffer) > 8192:
            del self.buffer[:len(self.buffer) - 8192]
        out = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != MAGIC:
                del self.buffer[0]
                continue
            if len(self.buffer) < 13:
                break
            if self.buffer[2] != VERSION:
                self.version_errors += 1
                del self.buffer[0]
                continue
            size = struct.unpack_from('<H', self.buffer, 7)[0]
            if size > MAX_PAYLOAD:
                self.length_errors += 1
                del self.buffer[0]
                continue
            total = 15 + size
            if len(self.buffer) < total:
                break
            got_crc = struct.unpack_from('<H', self.buffer, 13 + size)[0]
            if crc16(self.buffer[2:13 + size]) != got_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            sequence = struct.unpack_from('<H', self.buffer, 5)[0]
            timestamp = struct.unpack_from('<I', self.buffer, 9)[0]
            out.append(Frame(self.buffer[3], bytes(self.buffer[13:13 + size]),
                             sequence, timestamp, self.buffer[4]))
            del self.buffer[:total]
        return out


def pack_step(step):
    if step.kind not in KINDS or step.node not in NODE_NAMES:
        raise WireError('unknown simulation step')
    if step.cmd_id <= 0 or step.cmd_id > 0xFFFFFFFF:
        raise WireError('invalid cmd_id')
    values = (KINDS.index(step.kind) + 1, step.stage_index,
              NODE_NAMES.index(step.node), step.batch, step.color_id, step.ring_id)
    if any(type(v) is not int or not 0 <= v <= 255 for v in values):
        raise WireError('invalid step field')
    road = step.road_id.encode('ascii')
    if len(road) not in (0, 5):
        raise WireError('invalid road id length')
    if step.kind == 'MOVE_NODE':
        if not step.node or len(road) != 5:
            raise WireError('MOVE_NODE requires destination node and road')
    elif step.node or road:
        raise WireError('only MOVE_NODE may contain node or road')
    return struct.pack('<I6B', step.cmd_id, *values) + road


def decode_step(payload):
    if len(payload) not in (10, 15):
        raise WireError('unexpected step length')
    cmd_id, kind, stage, node, batch, color, ring = struct.unpack_from('<I6B', payload)
    if (cmd_id == 0 or not 1 <= kind <= len(KINDS) or not 1 <= stage <= 8
            or node >= len(NODE_NAMES) or batch > 2 or color > 6 or ring > 3):
        raise WireError('invalid step parameters')
    kind_name = KINDS[kind - 1]
    road = payload[10:].decode('ascii')
    if kind_name == 'MOVE_NODE':
        # Dijkstra encodes an UNDIRECTED road in canonical graph order.
        # For example C3 -> C2 uses road_id "C2-C3", not "C3-C2".
        # The destination may therefore be either end of the named road.
        if not node or len(road) != 5 or road[2] != '-':
            raise WireError('invalid move destination and road')
        start, end = road[:2], road[3:]
        if (start not in NODE_NAMES[1:] or end not in NODE_NAMES[1:]
                or start == end or NODE_NAMES[node] not in (start, end)):
            raise WireError('road endpoints do not contain move destination')
    elif node or road:
        raise WireError('unexpected node / road for manipulation')
    return {'cmd_id': cmd_id, 'kind': kind_name, 'stage': stage,
            'node': NODE_NAMES[node], 'batch': batch, 'color': color,
            'ring': ring, 'road': road}


def _write_all(fd, data):
    offset = 0
    until = time.monotonic() + 1.0
    while offset < len(data):
        try:
            offset += os.write(fd, data[offset:])
        except BlockingIOError:
            if time.monotonic() > until:
                raise WireError('PTY write timeout')
            select.select([], [fd], [], 0.02)


def _read_one(fd):
    ready, _, _ = select.select([fd], [], [], 0.05)
    if not ready:
        return b''
    try:
        data = os.read(fd, 32768)
    except BlockingIOError:
        return b''
    if not data:
        raise WireError('PTY peer closed')
    return data


def _recv(fd, parser):
    return parser.feed(_read_one(fd))


class MockPeer:
    """Never accepts production command types; deduplicates cmd_ids."""
    def __init__(self, *, fail_at=None, drop_ack_at=None, corrupt_qr=False):
        self.parser = Parser()
        self.fail_at = fail_at
        self.drop_ack_at = drop_ack_at
        self.corrupt_qr = corrupt_qr
        self.executions = 0
        self.seen = {}
        self.sequence = 0

    def hello(self):
        return encode(Frame(TEST_HELLO, MOCK_MARKER))

    def on_bytes(self, data, qr_code):
        outgoing = []
        for frame in self.parser.feed(data):
            if frame.msg_type != TEST_STEP:
                continue
            try:
                step = decode_step(frame.payload)
            except (WireError, UnicodeError):
                continue  # bad payload: no action and no ACK
            cmd_id = step['cmd_id']
            if cmd_id in self.seen:
                # Repeated command must not run again; replay the ACK only.
                outgoing.extend(self.seen[cmd_id][:1])
                continue
            if self.drop_ack_at == cmd_id:
                self.seen[cmd_id] = []
                continue
            out = [Frame(ACK, struct.pack('<IBB', cmd_id, TEST_STEP, 0)),
                   Frame(STARTED, struct.pack('<I', cmd_id))]
            if self.fail_at == cmd_id:
                out.append(Frame(FAILED, struct.pack('<IH', cmd_id, 42)))
            else:
                if step['kind'] == 'SCAN_QR':
                    qr = '000+000+000+000' if self.corrupt_qr else qr_code
                    encoded = qr.encode('ascii')
                    out.append(Frame(TEST_QR, struct.pack('<I', cmd_id) + encoded))
                out.append(Frame(COMPLETED, struct.pack('<IB', cmd_id, 0)))
            self.seen[cmd_id] = out
            self.executions += 1
            outgoing.extend(out)
        return b''.join(encode(f) for f in outgoing)


def _consume_feedback(frames, engine, current_id, now):
    qr_by_id = {}
    ignored = 0
    for f in frames:
        if f.msg_type == TEST_QR:
            if len(f.payload) != 19:
                ignored += 1
                continue
            cmd_id = struct.unpack_from('<I', f.payload)[0]
            try:
                text = f.payload[4:].decode('ascii')
            except UnicodeError:
                ignored += 1
                continue
            if cmd_id == current_id:
                qr_by_id[cmd_id] = text
            else:
                ignored += 1
            continue
        if f.msg_type not in (ACK, REJECTED, STARTED, PROGRESS, COMPLETED, FAILED):
            ignored += 1
            continue
        if len(f.payload) < 4:
            ignored += 1
            continue
        cmd_id = struct.unpack_from('<I', f.payload)[0]
        if f.msg_type == ACK:
            if len(f.payload) != 6 or f.payload[4:] != bytes((TEST_STEP, 0)):
                ignored += 1
                continue
            event = 'ACK'
        elif f.msg_type == REJECTED:
            if len(f.payload) != 6:
                ignored += 1
                continue
            event = 'REJECTED'
        elif f.msg_type == STARTED:
            if len(f.payload) != 4:
                ignored += 1
                continue
            continue  # progress indication is not terminal
        elif f.msg_type == PROGRESS:
            if len(f.payload) != 6:
                ignored += 1
            continue
        elif f.msg_type == COMPLETED:
            if len(f.payload) != 5 or f.payload[4] != 0:
                ignored += 1
                continue
            event = 'COMPLETED'
        else:
            if len(f.payload) != 6:
                ignored += 1
                continue
            event = 'FAILED'
        if not engine.feedback(cmd_id, event, now,
                               qr_text=qr_by_id.get(cmd_id) if event == 'COMPLETED' else None):
            ignored += 1
    return ignored


def run_loopback(steps, qr_code, *, fail_at=None, drop_ack_at=None,
                 corrupt_qr=False, corrupt_crc_at=None, duplicate_at=None):
    """Create both PTY endpoints internally. Caller cannot provide a device path."""
    engine = SimExecutor(steps, qr_code)
    peer = MockPeer(fail_at=fail_at, drop_ack_at=drop_ack_at, corrupt_qr=corrupt_qr)
    master, slave = pty.openpty()
    host_parser = Parser()
    ignored = 0
    try:
        tty.setraw(slave)
        os.set_blocking(master, False)
        os.set_blocking(slave, False)
        _write_all(master, peer.hello())
        greeting = _recv(slave, host_parser)
        if len(greeting) != 1 or greeting[0].msg_type != TEST_HELLO or greeting[0].payload != MOCK_MARKER:
            raise WireError('no explicit MOCK_ONLY handshake: refuse transmission')
        now = 0
        while engine.state not in ('DONE', 'FAULT'):
            step = steps[engine.index]
            intent = engine.issue(now)
            if intent is None or not intent['hardware_authorized'] is False:
                raise WireError('unexpected real-control intent')
            packet = bytearray(encode(Frame(TEST_STEP, pack_step(step),
                                            sequence=step.cmd_id % 65536, timestamp_ms=now)))
            if step.cmd_id == corrupt_crc_at:
                packet[-1] ^= 0x01
            _write_all(slave, packet)
            reply = peer.on_bytes(_read_one(master), qr_code)
            if reply:
                _write_all(master, reply)
            frames = _recv(slave, host_parser)
            ignored += _consume_feedback(frames, engine, step.cmd_id, now + 1)
            if step.cmd_id == duplicate_at:
                # Retransmit exactly the same command; peer must not execute twice.
                _write_all(slave, encode(Frame(TEST_STEP, pack_step(step))))
                reply = peer.on_bytes(_read_one(master), qr_code)
                if reply:
                    _write_all(master, reply)
                ignored += _consume_feedback(_recv(slave, host_parser), engine,
                                             step.cmd_id, now + 2)
            if engine.state in ('WAIT_ACK', 'WAIT_RESULT'):
                engine.tick(now + 30002)
            now += 31000
        result = engine.summary()
        result['wire'] = {'mode': 'ANONYMOUS_PTY_ONLY', 'hardware_authorized': False,
                          'frames_received_ok': host_parser.crc_errors == 0,
                          'mock_executions': peer.executions,
                          'mock_unique_command_ids': len(peer.seen),
                          'ignored_feedback': ignored,
                          'peer_crc_errors': peer.parser.crc_errors}
        return result
    finally:
        os.close(master)
        os.close(slave)


def main():
    parser = argparse.ArgumentParser(description='CAR-LINK V1 TEST-only PTY mission loopback')
    parser.add_argument('--start-zone', choices=('upper_right', 'lower_right'), default='upper_right')
    parser.add_argument('--task-code', default='156+123+516+231')
    parser.add_argument('--block', action='append', default=[])
    parser.add_argument('--fail-at', type=int)
    parser.add_argument('--drop-ack-at', type=int)
    parser.add_argument('--corrupt-crc-at', type=int)
    parser.add_argument('--duplicate-at', type=int)
    parser.add_argument('--corrupt-qr', action='store_true')
    args = parser.parse_args()
    from rule_mission_core import build_mission
    try:
        plan = build_mission(args.start_zone, args.block, provenance='DEMO_ONLY')
        if plan['status'] == 'NO_ROUTE':
            print(json.dumps({'state': 'NO_ROUTE', 'hardware_authorized': False}))
            return 3
        steps = compile_workflow(plan, args.task_code)
        result = run_loopback(steps, args.task_code, fail_at=args.fail_at,
                              drop_ack_at=args.drop_ack_at, corrupt_qr=args.corrupt_qr,
                              corrupt_crc_at=args.corrupt_crc_at,
                              duplicate_at=args.duplicate_at)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result['state'] == 'DONE' else 4
    except (MissionError, WireError, ValueError, OSError) as exc:
        parser.exit(2, 'ERROR: {}\n'.format(exc))


if __name__ == '__main__':
    raise SystemExit(main())
