"""OpenMV 与 STM32F4 的 ASCII 串口协议，默认 UART3 / 115200 / 8N1。

V4Link 用于 architecture-v2 的 @SEQ,LEN,BODY*XX 帧。以下为可选的旧 V5。
首批沿用 V5：R → #SEQ，#ARM,1 → #ARM_OK,1，#OBJ 后以 #TAKE、K 确认单件。
步骤 4–25：#AT,n → #OBS,n,t,动作,区域,颜色,环号,u,v；#USE,n,t 确认取用观测；
#RESULT,n,t,OK 或 FAIL 回报实际结果。#STATUS 查询步号、状态和计数。
每个 # 帧以换行结束。原 V5 接受独立的 R、K、B 字节。
"""
import time
from machine import UART
import config as C


class SerialLink:
    _SIMPLE = {
        'R': ('RESET',), '#RESET': ('RESET',), 'K': ('K',), 'B': ('B',),
        '#ARM,0': ('ARM', 0), '#ARM,1': ('ARM', 1), '#TAKE': ('TAKE',),
        '#STATUS': ('STATUS',), '#GROUP_DONE,1': ('GROUP_DONE', 1),
    }

    def __init__(self):
        self.uart = UART(C.UART_ID, C.UART_BAUD, bits=C.UART_BITS,
                         parity=C.UART_PARITY, stop=C.UART_STOP_BITS,
                         timeout=C.UART_READ_TIMEOUT_MS)
        self.rx = bytearray()
        self.drop = False

    def send(self, name, *fields):
        """编码并写完一帧。遇到短写，只补发尚未写出的字节。"""
        line = '#' + name
        if fields:
            line += ',' + ','.join(str(value) for value in fields)
        data = (line + '\n').encode('ascii')
        start = time.ticks_ms()
        offset = 0
        while offset < len(data):
            written = self.uart.write(data[offset:])
            if written:
                offset += written
            if offset < len(data) and time.ticks_diff(time.ticks_ms(), start) >= C.UART_WRITE_TIMEOUT_MS:
                raise OSError('UART TX timeout')

    @staticmethod
    def _decode(line):
        fixed = SerialLink._SIMPLE.get(line)
        if fixed is not None:
            return fixed
        fields = line.split(',')
        if len(fields) < 2 or len(fields[1]) > 2 or not fields[1].isdigit():
            return ('INVALID',)
        step = int(fields[1])
        if fields[0] == '#AT' and len(fields) == 2:
            return ('AT', step)
        if len(fields) in (3, 4) and len(fields[2]) <= 9 and fields[2].isdigit():
            token = int(fields[2])
            if fields[0] == '#USE' and len(fields) == 3:
                return ('USE', step, token)
            if fields[0] == '#RESULT' and len(fields) == 4:
                return ('RESULT', step, token, fields[3])
        return ('INVALID',)

    def poll(self):
        """接收不完整帧时保留已有字节；一次读取也可返回多条完整命令。"""
        count = self.uart.any()
        if not count:
            return []
        data = self.uart.read(min(count, C.UART_READ_CHUNK))
        commands = []
        for byte in data or b'':
            if byte in (10, 13):
                if self.rx and not self.drop:
                    commands.append(self._decode(self.rx.decode('ascii')))
                self.rx = bytearray()
                self.drop = False
            elif self.drop:
                continue
            elif not self.rx and byte in (82, 75, 66):
                commands.append(self._decode(chr(byte)))
            elif (not self.rx and byte != 35) or not 32 <= byte <= 126 or len(self.rx) >= C.UART_MAX_LINE:
                self.drop = True
            else:
                self.rx.append(byte)
        return commands


class V4Link:
    """USART3 帧。裸 R 只给紧接着的扫码请求建立新序号基线。"""
    def __init__(self):
        self.uart = UART(C.UART_ID, C.UART_BAUD, bits=C.UART_BITS,
                         parity=C.UART_PARITY, stop=C.UART_STOP_BITS,
                         timeout=C.UART_READ_TIMEOUT_MS)
        self.rx = None
        self.tx_seq = 0
        self.last_rx = None
        self.scan_prefix = False
        self.restart_scan = False

    @staticmethod
    def checksum(data):
        value = 0
        for byte in data:
            value ^= byte
        return value

    def encode(self, fields):
        body = ','.join(str(field) for field in fields)
        raw = body.encode('ascii')
        if not 1 <= len(raw) <= 128 or any(b < 32 or b > 126 or b in b'@#*' for b in raw):
            raise ValueError('invalid V4 body')
        header = ('%d,%d,' % (self.tx_seq, len(raw))).encode() + raw
        wire = b'@' + header + ('*%02X\n' % self.checksum(header)).encode()
        self.tx_seq = (self.tx_seq + 1) & 65535
        return wire

    def send(self, *fields):
        self.write(self.encode(fields))

    def finish(self, *fields):
        # Keep the result and terminal IDLE adjacent on the UART.
        self.write(self.encode(fields) + self.encode(('IDLE',)))

    def write(self, wire):
        start = time.ticks_ms()
        offset = 0
        while offset < len(wire):
            count = self.uart.write(wire[offset:])
            if count:
                offset += count
            if offset < len(wire) and time.ticks_diff(time.ticks_ms(), start) >= C.UART_WRITE_TIMEOUT_MS:
                raise OSError('V4 UART TX timeout')

    def decode(self, line):
        if len(line) < 8 or line[-3] != 42:
            return None
        raw = bytes(line)
        header = raw[:-3]
        check = raw[-2:]
        if any(b not in b'0123456789abcdefABCDEF' for b in check):
            return None
        if int(check, 16) != self.checksum(header):
            return None
        parts = header.decode('ascii').split(',', 2)
        if len(parts) != 3:
            return None
        seq, size, body = parts
        if not seq.isdigit() or len(seq) > 5 or not size.isdigit() or len(size) > 3:
            return None
        seq, size = int(seq), int(size)
        if seq > 65535 or not 1 <= size <= 128 or size != len(body) or '*' in body:
            return None
        fields = tuple(body.split(','))
        new_scan = (self.restart_scan and fields[:2] == ('JOB', '1') and
                    (len(fields) == 2 or (len(fields) == 3 and fields[2].isdigit()
                     and len(fields[2]) <= 5 and int(fields[2]) <= 65535)))
        if not new_scan and self.last_rx is not None:
            distance = (seq - self.last_rx) & 65535
            if not 0 < distance < 32768:
                return None
        self.last_rx = seq
        return ('SCAN_BEGIN',) + fields[1:] if new_scan else fields

    def poll(self):
        commands = []
        size = min(self.uart.any(), C.UART_READ_CHUNK)
        for byte in (self.uart.read(size) if size else b'') or b'':
            if byte == 64:  # @ always resynchronizes a partial frame
                self.rx = bytearray()
                self.restart_scan = self.scan_prefix
                self.scan_prefix = False
            elif self.rx is None:
                self.scan_prefix = byte == 82
            elif byte == 10:
                command = self.decode(self.rx)
                if command is not None:
                    commands.append(command)
                self.rx = None
                self.restart_scan = False
            elif byte == 13:
                continue
            elif byte < 32 or byte > 126 or byte == 35 or len(self.rx) >= 190:
                self.rx = None
                self.restart_scan = False
            else:
                self.rx.append(byte)
        return commands
