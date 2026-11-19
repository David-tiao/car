"""OpenMV 与 STM32F4 的 ASCII 串口协议，默认 UART3 / 115200 / 8N1。

首批沿用 V5：R → #SEQ，#ARM,1 → #ARM_OK,1，#OBJ 后以 #TAKE、K 确认单件。
步骤 4–25：#AT,n → #OBS,n,t,动作,区域,颜色,环号,u,v；#USE,n,t 确认取用观测；
#RESULT,n,t,OK 或 FAIL 回报实际结果。#STATUS 查询步号、状态和计数。
每个 # 帧以换行结束。原 V5 接受独立的 R、K、B 字节。
"""
import time
from machine import UART
import algorithm.openmv.openmv_all.config as C


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
