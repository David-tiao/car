#!/usr/bin/env python3
"""Incremental TEST-only CAR-LINK PTY mission session; no ROS or hardware access.

This class owns two *anonymous* PTY file descriptors. It uses the phase-2
CAR-LINK codec/MockPeer and SimExecutor, advancing one task command per tick.
Its pose is a discrete road-node PREVIEW, never odometry or localization.
"""
import os
import pty
import tty

from mission_execution import MissionError, SimExecutor, compile_workflow
from mission_wire_sim import (ACK, COMPLETED, FAILED, PROGRESS, REJECTED, STARTED,
                              Frame, MockPeer, Parser, TEST_HELLO, TEST_STEP,
                              MOCK_MARKER, WireError, _consume_feedback,
                              _read_one, _write_all, encode, pack_step)
from rule_mission_core import NODES


class PtyMissionSession:
    def __init__(self, plan, task_code, fail_at=None, drop_ack_at=None):
        self.steps = compile_workflow(plan, task_code)
        self.engine = SimExecutor(self.steps, task_code)
        start = plan.get('waypoints', [None])[0]
        if start not in NODES:
            raise MissionError('simulated start node missing from topology')
        self.start_node = start
        self.node = start
        self.position = tuple(NODES[start])
        self.trace = [self.position]
        self.last = {'cmd_id': 0, 'kind': 'READY', 'event': 'READY', 'stage_index': 0}
        self.events = []
        self.now_ms = 0
        self.peer = MockPeer(fail_at=fail_at, drop_ack_at=drop_ack_at)
        self.host_parser = Parser()
        self.master = self.slave = -1
        try:
            self.master, self.slave = pty.openpty()
            tty.setraw(self.slave)
            os.set_blocking(self.master, False)
            os.set_blocking(self.slave, False)
            _write_all(self.master, self.peer.hello())
            greeting = self._drain(self.slave, self.host_parser)
            if (len(greeting) != 1 or greeting[0].msg_type != TEST_HELLO
                    or greeting[0].payload != MOCK_MARKER):
                raise WireError('MOCK_ONLY handshake missing; refusing to run')
        except BaseException:
            self.close()
            raise

    def close(self):
        for key in ('master', 'slave'):
            fd = getattr(self, key, -1)
            if fd >= 0:
                os.close(fd)
                setattr(self, key, -1)

    @staticmethod
    def _drain(fd, parser):
        packets = []
        for _ in range(8):
            chunk = _read_one(fd)
            if chunk:
                packets.extend(parser.feed(chunk))
            if packets:
                break
        return packets

    @property
    def state(self):
        return self.engine.state

    def advance(self):
        """Process exactly one command over the PTY, returning mock feedback frames."""
        if self.state not in ('READY',):
            return []
        current = self.steps[self.engine.index]
        intent = self.engine.issue(self.now_ms)
        if not intent or intent.get('hardware_authorized') is not False:
            raise WireError('Non-test command blocked')
        self.last = {'cmd_id': current.cmd_id, 'kind': current.kind,
                     'event': 'WAIT_ACK', 'stage_index': current.stage_index,
                     'batch': current.batch, 'color_id': current.color_id,
                     'ring_id': current.ring_id, 'node': current.node}
        _write_all(self.slave, encode(Frame(TEST_STEP, pack_step(current),
                                             sequence=current.cmd_id % 65536,
                                             timestamp_ms=self.now_ms)))
        # The mock MCU owns its own parser and receives actual PTY bytes.
        reply = b''
        for _ in range(8):
            chunk = _read_one(self.master)
            if chunk:
                reply += self.peer.on_bytes(chunk, self.engine.expected_qr)
                break
        if reply:
            _write_all(self.master, reply)
        received = self._drain(self.slave, self.host_parser) if reply else []
        names = {ACK: 'ACK', REJECTED: 'REJECTED', STARTED: 'STARTED',
                 PROGRESS: 'PROGRESS', COMPLETED: 'COMPLETED', FAILED: 'FAILED'}
        this_events = []
        for f in received:
            if f.msg_type in names and len(f.payload) >= 4:
                cmd_id = int.from_bytes(f.payload[:4], 'little')
                if cmd_id == current.cmd_id:
                    this_events.append({'cmd_id': cmd_id, 'event': names[f.msg_type],
                                        'event_type': f.msg_type})
        _consume_feedback(received, self.engine, current.cmd_id, self.now_ms + 1)
        if self.engine.state in ('WAIT_ACK', 'WAIT_RESULT'):
            self.engine.tick(self.now_ms + 30001)
        if self.engine.state == 'FAULT':
            this_events.append({'cmd_id': current.cmd_id, 'event': 'FAULT',
                                'reason': self.engine.error})
        if self.engine.state != 'FAULT' and current.kind == 'MOVE_NODE':
            self.node = current.node
            self.position = tuple(NODES[self.node])
            self.trace.append(self.position)
        self.last['event'] = ('FAULT' if self.engine.state == 'FAULT' else
                              'COMPLETED' if self.engine.state in ('READY', 'DONE') else
                              self.engine.state)
        self.events.extend(this_events)
        self.now_ms += 31000
        return this_events

    def snapshot(self):
        result = self.engine.summary()
        return {'schema': 'car_test_mission_replay_v1', 'source_type': 'MOCK',
                'mode': 'SIM_ONLY', 'hardware_authorized': False,
                'state': result['state'], 'error': result['error'],
                'completed_steps': result['completed_steps'],
                'total_steps': result['total_steps'],
                'pick_count': result['pick_count'],
                'place_count': result['place_count'],
                'stack_count': result['stack_count'],
                'home_confirmed': result['home_confirmed'],
                'cmd_id': self.last['cmd_id'], 'kind': self.last['kind'],
                'event': self.last['event'], 'stage_index': self.last['stage_index'],
                'node': self.node,
                'x_m': round(self.position[0], 4),
                'y_m': round(self.position[1], 4),
                'pose_source': 'DISCRETE_ROAD_NODE',
                'mock_executions': self.peer.executions,
                'events': self.events[-16:]}
