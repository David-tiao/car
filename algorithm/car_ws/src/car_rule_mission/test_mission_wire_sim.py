#!/usr/bin/env python3
"""No-ROS tests for CAR-LINK framing and Linux PTY simulation."""
import struct
import unittest
from mission_execution import compile_workflow
from mission_wire_sim import (ACK, Frame, MockPeer, Parser, TEST_STEP,
                              crc16, decode_step, encode, pack_step, run_loopback)

CODE = '156+123+516+231'
ACTIONS = ('SCAN_QR', 'PICK_BATCH_1', 'ROUGH_PROCESS_BATCH_1',
           'STORE_BATCH_1', 'PICK_BATCH_2', 'ROUGH_PROCESS_BATCH_2',
           'STACK_BATCH_2', 'RETURN_START')
NODES = ('C3', 'C2', 'B3', 'B1', 'A2', 'B3', 'B1', 'A2', 'C3')


def fake_steps():
    legs = []
    for i, (a, b) in enumerate(zip(NODES, NODES[1:]), 1):
        legs.append({'stage_index': i, 'action': ACTIONS[i - 1],
                     'start_node': a, 'goal_node': b,
                     'node_path': [a, b], 'road_ids': [a+'-'+b]})
    plan = {'status': 'DEMO_ROUTE_ONLY', 'commands_enabled': False,
            'NOT_FOR_NAV2_FOLLOWPATH': True, 'legs': legs, 'blocked_roads': []}
    return compile_workflow(plan, CODE)


class WireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.steps = fake_steps()

    def test_crc_vector_and_known_carlink_frame(self):
        self.assertEqual(crc16(b'123456789'), 0x29B1)
        pkt = encode(Frame(TEST_STEP, b'ABCD', sequence=7, timestamp_ms=1234))
        self.assertEqual(pkt[:13], bytes.fromhex('a5 5a 01 e0 00 07 00 04 00 d2 04 00 00'))
        self.assertEqual(struct.unpack_from('<H', pkt, len(pkt)-2)[0], crc16(pkt[2:-2]))
        self.assertEqual(len(pkt), 19)

    def test_parser_split_noise_corruption_and_length(self):
        pkt = encode(Frame(ACK, b'\x01\x00\x00\x00\xe0\x00'))
        p = Parser()
        self.assertEqual(p.feed(b'\x00\x01' + pkt[:8]), [])
        self.assertEqual(p.feed(pkt[8:]), [Frame(ACK, b'\x01\x00\x00\x00\xe0\x00')])
        corrupt = bytearray(pkt)
        corrupt[-1] ^= 1
        self.assertEqual(p.feed(corrupt), [])
        self.assertEqual(p.crc_errors, 1)
        bad = bytearray(pkt)
        bad[7:9] = b'\xff\xff'
        p.feed(bad)
        self.assertGreaterEqual(p.length_errors, 1)

    def test_canonical_road_accepts_reverse_direction(self):
        # Route C3 -> C2 legally travels canonical edge "C2-C3".
        from dataclasses import replace
        original = next(s for s in self.steps
                        if s.kind == 'MOVE_NODE' and s.node == 'C2')
        step = replace(original, road_id='C2-C3')
        decoded = decode_step(pack_step(step))
        self.assertEqual(decoded['node'], 'C2')
        self.assertEqual(decoded['road'], 'C2-C3')
        # A destination outside the road's endpoints must still be rejected.
        from mission_wire_sim import WireError
        with self.assertRaises(WireError):
            decode_step(pack_step(replace(step, node='A1')))

    def test_step_roundtrip(self):
        for step in self.steps:
            got = decode_step(pack_step(step))
            self.assertEqual(got['cmd_id'], step.cmd_id)
            self.assertEqual(got['kind'], step.kind)
            self.assertEqual(got['node'], step.node)

    def test_roundtrip_done_and_counted(self):
        r = run_loopback(self.steps, CODE)
        self.assertEqual(r['state'], 'DONE')
        self.assertEqual((r['pick_count'], r['place_count'], r['stack_count']), (12, 12, 3))
        self.assertTrue(r['home_confirmed'])
        self.assertEqual(r['wire']['mock_executions'], len(self.steps))
        self.assertFalse(r['wire']['hardware_authorized'])

    def test_deduplicated_command(self):
        r = run_loopback(self.steps, CODE, duplicate_at=3)
        self.assertEqual(r['state'], 'DONE')
        self.assertEqual(r['wire']['mock_executions'], len(self.steps))
        self.assertGreater(r['wire']['ignored_feedback'], 0)

    def test_failure_latches_and_no_subsequent_command(self):
        r = run_loopback(self.steps, CODE, fail_at=3)
        self.assertEqual(r['state'], 'FAULT')
        self.assertEqual(r['wire']['mock_executions'], 3)

    def test_dropped_ack_latches_timeout(self):
        r = run_loopback(self.steps, CODE, drop_ack_at=2)
        self.assertEqual(r['state'], 'FAULT')
        self.assertTrue(r['error'].startswith('TIMEOUT'))
        self.assertEqual(r['wire']['mock_executions'], 1)

    def test_corrupted_command_crc_discarded(self):
        r = run_loopback(self.steps, CODE, corrupt_crc_at=2)
        self.assertEqual(r['state'], 'FAULT')
        self.assertGreaterEqual(r['wire']['peer_crc_errors'], 1)

    def test_mismatched_qr_never_allows_pick(self):
        r = run_loopback(self.steps, CODE, corrupt_qr=True)
        self.assertEqual(r['state'], 'FAULT')
        self.assertEqual(r['pick_count'], 0)
        self.assertIn('QR_MISMATCH', r['error'])

    def test_mock_disallows_production_position_frame(self):
        peer = MockPeer()
        normal_position = encode(Frame(0x83, struct.pack('<I', 1)))
        self.assertEqual(peer.on_bytes(normal_position, CODE), b'')
        self.assertEqual(peer.executions, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
