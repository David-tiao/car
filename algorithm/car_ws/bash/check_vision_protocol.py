#!/usr/bin/env python3
"""OpenMV / STM32F4 视觉协议静态验收，只读。"""

import argparse
import hashlib
import json
import re
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

V5 = ROOT / "algorithm/openmv/openmv_all"
V4 = ROOT / "STM32F4/vision_openmv_stm32_v4_redownload/OpenMV"
MCU = ROOT / "STM32F4/Hardware"


def read(path):
    data = path.read_bytes()
    return (
        data.decode("utf-8"),
        hashlib.sha256(data).hexdigest()[:16]
    )


def frame(sequence, body):
    """按照 STM32F4 V4 格式构造测试帧。"""
    payload = body.encode("ascii")

    if not 0 <= sequence <= 65535:
        raise ValueError("Invalid sequence")

    if not 1 <= len(payload) <= 128:
        raise ValueError("Invalid body length")

    if any(
        x < 32 or x > 126 or x in (42, 64)
        for x in payload
    ):
        raise ValueError("Invalid body character")

    header = (
        f"{sequence},{len(payload)},{body}"
    ).encode("ascii")

    crc = 0
    for byte in header:
        crc ^= byte

    return (
        b"@" + header +
        f"*{crc:02X}\n".encode("ascii")
    )


def unframe(data):
    """纯 Python 参考解码，用于格式单元测试。"""
    if (
        not isinstance(data, bytes)
        or not data.startswith(b"@")
        or not data.endswith(b"\n")
    ):
        raise ValueError("Invalid frame boundary")

    text = data[1:-1].decode("ascii")

    if text.count("*") != 1:
        raise ValueError("Invalid checksum marker")

    header, tag = text.split("*")

    if (
        len(tag) != 2
        or not re.fullmatch(r"[0-9A-Fa-f]{2}", tag)
    ):
        raise ValueError("Invalid checksum")

    checksum = 0
    for byte in header.encode("ascii"):
        checksum ^= byte

    if checksum != int(tag, 16):
        raise ValueError("Checksum mismatch")

    seq, length, body = header.split(",", 2)

    if (
        not seq.isdigit()
        or not length.isdigit()
        or int(seq) > 65535
    ):
        raise ValueError("Invalid sequence/length")

    actual = len(body.encode("ascii"))

    if not 1 <= actual <= 128 or int(length) != actual:
        raise ValueError("Body length mismatch")

    return int(seq), body


def selftest():
    messages = [
        "READY,IDLE",
        "ACK,JOB,1",
        "POS,RING,1,5,-3,160,120,9,2",
        "QR,123+123+123+123",
    ]

    for seq, body in enumerate(messages):
        assert unframe(frame(seq, body)) == (seq, body)

    raw = frame(9, "READY,IDLE")

    corrupt = (
        raw[:-3] + b"00\n"
        if raw[-3:-1] != b"00"
        else raw[:-3] + b"FF\n"
    )

    malformed = [
        corrupt,
        raw.replace(b",10,", b",11,"),
        raw[:-1],
    ]

    for item in malformed:
        try:
            unframe(item)
        except ValueError:
            continue
        raise AssertionError("Invalid frame accepted")

    # 16 位序号回绕和旧序号判定示例
    assert ((0 - 65535) & 65535) == 1
    assert ((99 - 100) & 65535) >= 32768

    return messages


def check(name, success, note=""):
    return {
        "name": name,
        "pass": bool(success),
        "note": note,
    }


def audit_openmv(folder, variant, mcu_commands):
    files = [
        folder / "main.py",
        folder / "config.py",
        folder / (
            "serial.py" if variant == "v5"
            else "utils.py"
        ),
    ]

    contents = {}
    hashes = {}

    for path in files:
        contents[path.name], digest = read(path)
        hashes[path.name] = {
            "path": str(path),
            "sha256_prefix": digest,
        }

    main = contents["main.py"]
    config = contents["config.py"]

    serial_name = (
        "serial.py" if variant == "v5"
        else "utils.py"
    )
    serial = contents[serial_name]

    checks = [
        check(
            "USART3 / 115200",
            bool(re.search(
                r"(?m)^UART_ID\s*=\s*3\s*$",
                config
            )) and bool(re.search(
                r"(?m)^UART_BAUD\s*=\s*115200\s*$",
                config
            )),
        ),
    ]

    if variant == "v4":
        checks.extend([
            check(
                "使用 PacketSerial",
                "serial = U.PacketSerial()" in main
                and "class PacketSerial:" in serial,
            ),
            check(
                "帧头/长度/XOR 格式",
                "header = '%d,%d,%s'" in serial
                and "@%s*%02X" in serial
                and "crc ^= ord(ch)" in serial,
            ),
            check(
                "16 位序号防重复处理",
                "def _accept_rx_seq(" in serial
                and "distance == 0 or distance >= 32768"
                in serial,
            ),
            check(
                "READY 与 JOB ACK",
                "serial.send('READY', 'IDLE')" in main
                and "serial.send('ACK', 'JOB', code)"
                in main,
            ),
            check(
                "ALIGN 会话确认",
                "command == 'ALIGN'" in main
                and "serial.send('ACK', 'ALIGN', target, session)"
                in main,
            ),
        ])

    else:
        emitted = sorted(set(re.findall(
            r"""(?:self\.)?link\.send\(\s*['"]([A-Z0-9_]+)""",
            main
        )))

        checks.extend([
            check(
                "V5 使用 STM32F4 V4 帧格式",
                "class PacketSerial:" in serial
                and "@%s*%02X" in serial,
                "当前 V5 serial.py 使用 #消息名 文本帧",
            ),
            check(
                "旧 #SEQ 二维码兼容路径",
                "self.link.send('SEQ', *''.join(self.job.groups))"
                in main
                and "line = '#' + name" in serial,
                "仅代表旧 #SEQ 消息结构符合兼容路径",
            ),
            check(
                "V5 观测/确认消息已被 STM32 解析",
                all(
                    name in mcu_commands
                    for name in (
                        "OBS", "USE_OK", "RESULT_OK"
                    )
                ),
                "STM32 V4 尚未实现整套 V5 观测交互",
            ),
        ])

        hashes["emitted_messages"] = emitted

    passed = all(row["pass"] for row in checks)

    return {
        "variant": variant,
        "status": (
            "STATIC_PASS" if passed
            else "INCOMPATIBLE"
        ),
        "sources": hashes,
        "checks": checks,
    }


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--variant",
        choices=("both", "v4", "v5"),
        default="both",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
    )

    args = parser.parse_args()

    examples = selftest()

    if args.self_test:
        print("[PASS] V4 帧结构、校验及异常帧测试")
        return 0

    try:
        vision, vision_hash = read(MCU / "vision.c")
        header, header_hash = read(MCU / "vision.h")

        mcu_checks = [
            check(
                "STM32 使用 @ 帧头及 XOR",
                "line[0] != '@'" in vision
                and "xor_bytes(line + 1U" in vision,
            ),
            check(
                "STM32 校验长度与序号",
                "strlen(*body) != parsed_length" in vision
                and "newer_sequence(sequence" in vision,
            ),
            check(
                "STM32 保留旧 #SEQ 兼容",
                "parse_legacy_qr" in vision
                and 'strncmp(line, "#SEQ,"' in vision,
            ),
            check(
                "STM32 body 上限为 128 字节",
                bool(re.search(
                    r"VISION_BODY_MAX\s+128U",
                    header
                )),
            ),
        ]

        accepted = set(re.findall(
            r'!strcmp\(fields\[0\],\s*"([A-Z_]+)"',
            vision
        ))

        selected = (
            ("v5", "v4")
            if args.variant == "both"
            else (args.variant,)
        )

        variants = [
            audit_openmv(
                V5 if variant == "v5" else V4,
                variant,
                accepted,
            )
            for variant in selected
        ]

    except (OSError, UnicodeError) as exc:
        print("[FAIL] 无法读取源码:", exc)
        return 2

    passed = (
        all(row["pass"] for row in mcu_checks)
        and all(
            item["status"] == "STATIC_PASS"
            for item in variants
        )
    )

    report = {
        "schema": "car_vision_protocol_audit_v1",
        "timestamp": datetime.now().isoformat(),
        "mode": args.variant,
        "status": (
            "STATIC_PASS" if passed
            else "INCOMPATIBLE"
        ),
        "scope": "静态源码检查，未连接真实硬件",
        "stm32": {
            "vision_c_sha256_prefix": vision_hash,
            "vision_h_sha256_prefix": header_hash,
            "accepted_messages": sorted(accepted),
            "checks": mcu_checks,
        },
        "openmv": variants,
        "example_frames": [
            frame(i, body).decode("ascii")
            for i, body in enumerate(examples)
        ],
        "recommendation": (
            "STM32F4 保持不变；OpenMV 使用 V4 帧协议，"
            "其余视觉业务消息需逐项适配。"
        ),
    }

    directory = (
        ROOT / "sim2real/data/web_modes"
    )
    directory.mkdir(parents=True, exist_ok=True)

    output = directory / (
        "vision_protocol_audit_"
        + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        + ".json"
    )

    output.write_text(
        json.dumps(
            report,
            ensure_ascii=False,
            indent=2
        ) + "\n",
        encoding="utf-8",
    )

    print("\n=== STM32F4 标准 ===")

    for item in mcu_checks:
        print(
            "[PASS]" if item["pass"] else "[FAIL]",
            item["name"],
        )

    for variant in variants:
        print(
            f"\n=== OpenMV {variant['variant']}: "
            f"{variant['status']} ==="
        )
        for item in variant["checks"]:
            print(
                "[PASS]" if item["pass"] else "[FAIL]",
                item["name"],
            )
            if not item["pass"] and item["note"]:
                print("  ", item["note"])

    print("\n[REPORT]", output)
    print(
        "[INFO] STATIC_PASS 仅代表源码静态格式检查通过，"
        "不代表两块真实开发板已经联调成功"
    )

    return 0 if passed else 2


if __name__ == "__main__":
    sys.exit(main())
