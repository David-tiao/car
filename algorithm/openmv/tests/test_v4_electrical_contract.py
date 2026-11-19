#!/usr/bin/env python3
"""Host-only contract tests for architecture-v2 OpenMV V4 camera firmware.

Uses AST extraction for pure Python functions/classes; never imports OpenMV
hardware modules (csi, display, image, machine) or accesses real UART.
"""
import ast
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
CAMERA = ROOT / "openmv_v4"
MIRROR = ROOT.parents[1] / "STM32F4" / "vision_openmv_stm32_v4_redownload" / "OpenMV"


def extract(path, symbols, environment):
    module = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    selected = [node for node in module.body
                if isinstance(node, (ast.FunctionDef, ast.ClassDef))
                and node.name in symbols]
    found = {node.name for node in selected}
    if found != set(symbols):
        raise AssertionError("Missing expected declarations {} in {}".format(set(symbols)-found, path))
    code = compile(ast.Module(body=selected, type_ignores=[]), str(path), "exec")
    exec(code, environment)
    return environment


class UARTFake:
    def __init__(self, *args, **kwargs):
        self.rx = bytearray()
        self.tx = []
        self.args = args
        self.kwargs = kwargs

    def any(self):
        return len(self.rx)

    def read(self, count):
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result

    def write(self, data):
        self.tx.append(bytes(data))
        return len(data)

    def inject(self, data):
        self.rx.extend(data)


def frame(seq, body):
    """Independent host encoder; matches reference V4 ASCII XOR over header."""
    if not 0 <= seq <= 65535:
        raise ValueError("invalid sequence")
    header = "{},{},{}".format(seq, len(body), body)
    crc = 0
    for byte in header.encode("ascii"):
        crc ^= byte
    return "@{}*{:02X}\n".format(header, crc).encode("ascii")


def v4_camera():
    config = {}
    exec(compile(CAMERA.joinpath("config.py").read_text(encoding="utf-8"),
                 str(CAMERA / "config.py"), "exec"), config)
    cfg = SimpleNamespace(**{k: v for k, v in config.items() if k.isupper()})
    helpers = extract(CAMERA / "utils.py",
                      {"PacketSerial", "normalize_code", "parse_code", "TaskBoard"},
                      {"UART": UARTFake, "C": cfg})
    main = extract(CAMERA / "main.py", {"Job", "decode_job"},
                   {"C": cfg})
    return cfg, helpers, main


class V4ContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config, cls.helpers, cls.main = v4_camera()

    def fresh_serial(self):
        return self.helpers["PacketSerial"]()

    def test_reference_deployed_v4_mirror_is_byte_identical(self):
        for name in ("main.py", "config.py", "utils.py"):
            with self.subTest(name=name):
                self.assertEqual((CAMERA / name).read_bytes(),
                                 (MIRROR / name).read_bytes())

    def test_expected_uart_and_calibration_guards(self):
        c = self.config
        self.assertEqual((c.UART_ID, c.UART_BAUD), (3, 115200))
        self.assertIs(c.JOB_QR_TIMEOUT_MS, None)
        self.assertFalse(c.GRIP_TARGET_CALIBRATED)
        self.assertFalse(c.DISTANCE_CALIBRATED)
        self.assertEqual(c.FX_PX, 0.0)
        self.assertEqual(c.FY_PX, 0.0)

    def test_reference_known_align_frame(self):
        self.assertEqual(frame(1, "ALIGN,2,7"), b"@1,9,ALIGN,2,7*40\n")
        link = self.fresh_serial()
        link.uart.inject(b"@1,9,ALIGN,2,7*40\n")
        self.assertEqual(link.poll(), [["ALIGN", "2", "7"]])

    def test_ready_and_qr_with_session_roundtrip(self):
        link = self.fresh_serial()
        self.assertTrue(link.send("READY", "IDLE"))
        self.assertEqual(link.uart.tx[-1], frame(0, "READY,IDLE"))
        link.uart.inject(frame(5, "JOB,1,23"))
        self.assertEqual(link.poll(), [["JOB", "1", "23"]])
        self.assertTrue(link.send("ACK", "JOB", 1))
        self.assertTrue(link.send("QR", "156+123+516+231", 23))
        self.assertEqual(link.uart.tx[-1], frame(2, "QR,156+123+516+231,23"))
        link.uart.inject(frame(6, "ACK,QR,23"))
        self.assertEqual(link.poll(), [["ACK", "QR", "23"]])
        self.assertTrue(link.send("DONE", "QR"))
        self.assertTrue(link.send("IDLE"))

    def test_incremental_and_multiple_frames(self):
        link = self.fresh_serial()
        first = frame(5, "STATUS")
        link.uart.inject(first[:6])
        self.assertEqual(link.poll(), [])
        link.uart.inject(first[6:] + frame(6, "CANCEL"))
        self.assertEqual(link.poll(), [["STATUS"], ["CANCEL"]])

    def test_reject_duplicate_old_wraparound(self):
        link = self.fresh_serial()
        link.uart.inject(frame(65535, "STATUS"))
        self.assertEqual(link.poll(), [["STATUS"]])
        link.uart.inject(frame(65535, "STATUS") + frame(0, "CANCEL"))
        self.assertEqual(link.poll(), [["CANCEL"]])
        link.uart.inject(frame(65535, "STATUS"))
        self.assertEqual(link.poll(), [])

    def test_bad_xor_length_and_oversized_frame_do_not_execute(self):
        link = self.fresh_serial()
        bad = bytearray(frame(0, "JOB,42"))
        bad[-3] = ord("0") if bad[-3] != ord("0") else ord("1")
        link.uart.inject(b"noise" + bytes(bad) + b"@1,255,JOB,42*00\n")
        self.assertEqual(link.poll(), [])
        self.assertGreaterEqual(link.bad, 2)
        link.uart.inject(b"@" + b"X"*200 + b"\n" + frame(2, "STATUS"))
        self.assertEqual(link.poll(), [["STATUS"]])

    def test_qr_sessions_are_parsed_and_not_deadlined_locally(self):
        decoder = self.main["decode_job"]
        job_code, qr = decoder(["JOB", "1", "7"], 100, self.config)
        self.assertEqual(job_code, 1)
        self.assertEqual(qr.kind, "QR")
        self.assertEqual(qr.qr_session, 7)
        self.assertIsNone(qr.timeout_ms)
        job_code, legacy = decoder(["JOB", "1"], 100, self.config)
        self.assertEqual(legacy.qr_session, 0)
        with self.assertRaises(ValueError):
            decoder(["JOB", "1", "65536"], 100, self.config)
        with self.assertRaises(ValueError):
            decoder(["JOB", "11", "8"], 100, self.config)

    def test_ring_and_material_job_identifiers(self):
        parser = self.main["decode_job"]
        for code, kind, target in ((21, "RING", 1), (23, "RING", 3),
                                   (40, "AREA", 0), (41, "MATERIAL_ALIGN", 0),
                                   (42, "MATERIAL_PICK", 0)):
            with self.subTest(code=code):
                n, job = parser(["JOB", str(code)], 0, self.config)
                self.assertEqual((n, job.kind, job.target), (code, kind, target))

    def test_task_code_and_board(self):
        parse = self.helpers["parse_code"]
        self.assertEqual(parse("156+123+516+231"),
                         ["156", "123", "516", "231"])
        for bad in ("156+123+516", "156+112+516+231", "156+123+516+239",
                    "156+123+516+231+1"):
            with self.subTest(bad=bad):
                self.assertIsNone(parse(bad))
        board = self.helpers["TaskBoard"]()
        self.assertTrue(board.accept("156+123+516+231"))
        self.assertFalse(board.accept("156+123+516+231"))
        self.assertTrue(board.finish_group(1))
        self.assertFalse(board.finish_group(3))

    def test_command_driven_qr_session_flow_is_present(self):
        src = (CAMERA / "main.py").read_text(encoding="utf-8")
        for fragment in ("'QR', found, job.qr_session",
                         "'ACK' and len(packet) == 3",
                         "int(packet[2]) != job.qr_session",
                         "'DONE', 'QR'",
                         "'PICK_READY'",
                         "'ALIGN'",
                         "'STEP_DONE'"):
            self.assertIn(fragment, src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
