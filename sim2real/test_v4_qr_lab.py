"""TEST-only OpenMV V4 ↔ STM32 QR session protocol experiment.

This is a deterministic *contract model*, NOT the compiled STM32 race firmware.
No serial devices, ROS commands, actuator commands or /api/real state are used.
The matching STM32 source is verified separately by GCC tests/race.
"""
from __future__ import annotations

import ipaddress
import json
import threading
from urllib.parse import urlsplit

SCENARIOS = {
    "success": "正常扫码与确认",
    "bad_xor": "二维码帧 XOR 损坏",
    "stale_session": "旧扫码会话号",
    "duplicate_seq": "相同帧序号重复发送",
    "invalid_task": "任务码不合法",
    "missing_job_ack": "启动 ACK 丢失",
    "missing_qr": "二维码一直未到达",
    "cancel": "扫描过程中 CANCEL",
}
DEFAULT_CODE = "156+123+516+231"
LOCK = threading.RLock()
LAST = None


def valid_task(code):
    if not isinstance(code, str):
        return False
    g = code.split("+")
    return (len(g) == 4
            and all(len(x) == 3 and len(set(x)) == 3
                    and all(c in ("123456" if i % 2 == 0 else "123") for c in x)
                    for i, x in enumerate(g))
            and set(g[0]) == set(g[2]))


def frame(seq, body):
    if type(seq) is not int or not 0 <= seq <= 65535:
        raise ValueError("Invalid sequence")
    if not isinstance(body, str) or not body.isascii() or not 1 <= len(body) <= 128:
        raise ValueError("Invalid V4 body")
    if any(c in body for c in "@*\r\n"):
        raise ValueError("Forbidden V4 body byte")
    header = "{},{},{}".format(seq, len(body), body)
    checksum = 0
    for c in header.encode("ascii"):
        checksum ^= c
    return "@{}*{:02X}\n".format(header, checksum)


def decode(packet):
    """Validate the same framing invariants as STM32 vision.c, independently."""
    if not isinstance(packet, str) or not packet.startswith("@") or not packet.endswith("\n"):
        raise ValueError("Missing V4 frame boundaries")
    data = packet[1:-1]
    if data.count("*") != 1:
        raise ValueError("Invalid V4 checksum delimiter")
    header, check = data.split("*")
    if len(check) != 2 or any(x not in "0123456789abcdefABCDEF" for x in check):
        raise ValueError("Invalid XOR encoding")
    checksum = 0
    try:
        raw = header.encode("ascii")
    except UnicodeError as exc:
        raise ValueError("Non-ASCII wire data") from exc
    for v in raw:
        checksum ^= v
    if checksum != int(check, 16):
        raise ValueError("XOR mismatch")
    parts = header.split(",", 2)
    if len(parts) != 3 or not parts[0].isdigit() or not parts[1].isdigit():
        raise ValueError("Invalid V4 header")
    seq, length = int(parts[0]), int(parts[1])
    body = parts[2]
    if not 0 <= seq <= 65535 or not 1 <= length <= 128 or len(body) != length:
        raise ValueError("Invalid V4 sequence or LEN")
    if any(c in body for c in "@*\r\n"):
        raise ValueError("Invalid V4 body")
    return seq, body


class QrExperiment:
    def __init__(self, session=23):
        self.session = session
        self.state = "IDLE"
        self.last_rx = None
        self.tx_seq = 0
        self.rx_seq = 0
        self.task_code = ""
        self.accepted_qr = 0
        self.events = []

    def log(self, direction, body, wire, accepted=True, note=""):
        self.events.append({"direction": direction, "body": body, "wire": wire,
                            "state": self.state, "accepted": accepted, "note": note})

    def tx(self, body):
        wire = frame(self.tx_seq, body)
        self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        self.log("STM32 → OpenMV", body, wire.strip())
        return wire

    def begin(self):
        if self.state != "IDLE":
            raise ValueError("Already started")
        self.log("STM32 → OpenMV", "R (legacy reset)", "R", note="Compatibility prefix")
        self.state = "WAIT_ACK"
        self.tx("JOB,1,{}".format(self.session))

    def receive(self, body, seq=None, corrupt=False):
        if seq is None:
            seq = self.rx_seq
            self.rx_seq = (self.rx_seq + 1) & 0xFFFF
        wire = frame(seq, body)
        if corrupt:
            wire = wire[:-3] + ("0" if wire[-3] != "0" else "1") + wire[-2:]
        try:
            decoded_seq, parsed_body = decode(wire)
        except ValueError as exc:
            self.log("OpenMV → STM32", body, wire.strip(), False, str(exc))
            return False
        if self.last_rx is not None:
            diff = (decoded_seq - self.last_rx) & 0xFFFF
            if diff == 0 or diff >= 0x8000:
                self.log("OpenMV → STM32", body, wire.strip(), False, "DUPLICATE_OR_STALE_SEQ")
                return False
        self.last_rx = decoded_seq
        if self.state == "WAIT_ACK" and parsed_body == "ACK,JOB,1":
            self.state = "RUNNING"
            self.log("OpenMV → STM32", body, wire.strip(), True)
            return True
        if self.state == "RUNNING" and parsed_body.startswith("QR,"):
            parts = parsed_body.split(",")
            matched = (len(parts) == 3 and parts[2].isdigit()
                       and int(parts[2]) == self.session and valid_task(parts[1]))
            if matched:
                self.task_code = parts[1]
                self.accepted_qr += 1
                self.state = "QR_ACCEPTED"
            self.log("OpenMV → STM32", body, wire.strip(), matched,
                     "" if matched else "INVALID_QR_OR_SESSION")
            return matched
        if self.state == "QR_ACCEPTED" and parsed_body.startswith("QR,"):
            self.log("OpenMV → STM32", body, wire.strip(), False, "QR_ALREADY_ACCEPTED")
            return False
        if self.state == "WAIT_DONE" and parsed_body == "DONE,QR":
            self.log("OpenMV → STM32", body, wire.strip())
            self.state = "WAIT_IDLE"
            return True
        if self.state == "WAIT_IDLE" and parsed_body == "IDLE":
            self.state = "DONE"
            self.log("OpenMV → STM32", body, wire.strip())
            return True
        self.log("OpenMV → STM32", body, wire.strip(), False, "UNEXPECTED_FOR_STATE")
        return False

    def ack_qr(self):
        if self.state != "QR_ACCEPTED":
            return False
        self.tx("ACK,QR,{}".format(self.session))
        self.state = "WAIT_DONE"
        return True

    def timeout(self):
        if self.state != "DONE":
            self.state = "FAULT"
            self.log("STM32", "SCAN_TIMEOUT", "", False, "No valid QR and ACK completion")

    def cancel(self):
        self.tx("CANCEL")
        self.state = "CANCELLED"
        self.log("STM32", "CANCELLED", "", False, "No motion permitted")


def run_case(scenario):
    if scenario not in SCENARIOS:
        raise ValueError("Unknown TEST scenario")
    s = QrExperiment()
    s.begin()
    if scenario == "missing_job_ack":
        s.timeout()
    else:
        s.receive("ACK,JOB,1")
        if scenario == "cancel":
            s.cancel()
        elif scenario == "missing_qr":
            s.timeout()
        elif scenario == "bad_xor":
            s.receive("QR,{},23".format(DEFAULT_CODE), corrupt=True)
            s.timeout()
        elif scenario == "stale_session":
            s.receive("QR,{},22".format(DEFAULT_CODE))
            s.timeout()
        elif scenario == "invalid_task":
            s.receive("QR,156+112+516+231,23")
            s.timeout()
        else:
            qr_body = "QR,{},23".format(DEFAULT_CODE)
            accepted_seq = s.rx_seq
            assert s.receive(qr_body)
            if scenario == "duplicate_seq":
                assert not s.receive(qr_body, seq=accepted_seq)
            assert s.ack_qr()
            assert s.receive("DONE,QR")
            assert s.receive("IDLE")
    return {"schema": "car_v4_qr_lab_v1",
            "source_type": "MOCK", "mode": "SIM_ONLY",
            "hardware_authorized": False,
            "model_kind": "CONTRACT_MODEL_NOT_STM32_FIRMWARE",
            "scenario": scenario, "label": SCENARIOS[scenario],
            "session": s.session, "task_code": s.task_code,
            "state": s.state, "accepted_qr": s.accepted_qr,
            "events": s.events,
            "passed": ((s.state == "DONE" and s.accepted_qr == 1)
                       if scenario in ("success", "duplicate_seq") else
                       (s.state == ("CANCELLED" if scenario == "cancel" else "FAULT")
                        and s.accepted_qr == 0))}


def snapshot():
    with LOCK:
        return {"source_type": "MOCK", "hardware_authorized": False,
                "scenarios": SCENARIOS, "snapshot": LAST}


def handle_get(handler):
    if urlsplit(handler.path).path != "/api/test/v4_qr_lab":
        return False
    handler.respond_json(200, snapshot())
    return True


def handle_post(handler):
    if urlsplit(handler.path).path != "/api/test/v4_qr_lab":
        return False
    try:
        if not ipaddress.ip_address(handler.client_address[0]).is_loopback:
            handler.respond_json(403, {"error": "Only localhost TEST experiments allowed"})
            return True
    except ValueError:
        handler.respond_json(403, {"error": "Invalid source"})
        return True
    origin = handler.headers.get("Origin")
    if origin and urlsplit(origin).netloc.lower() != handler.headers.get("Host", "").lower():
        handler.respond_json(403, {"error": "Cross-origin TEST command forbidden"})
        return True
    try:
        if handler.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
            raise ValueError("JSON required")
        size = int(handler.headers.get("Content-Length", "0"))
        if not 0 < size <= 128:
            raise ValueError("Invalid TEST request size")
        payload = json.loads(handler.rfile.read(size),
                             parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite")))
        if not isinstance(payload, dict) or set(payload) != {"scenario"}:
            raise ValueError("Only scenario is accepted")
        report = run_case(payload["scenario"])
    except (TypeError, ValueError, UnicodeError):
        handler.respond_json(400, {"error": "Invalid TEST scenario"})
        return True
    global LAST
    with LOCK:
        LAST = report
    handler.respond_json(200, report)
    return True
