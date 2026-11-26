"""Linux POSIX tty transport, explicit open only; no automatic reconnect."""
import os
import errno
import fcntl
import termios
import stat


class PosixSerial:
    def __init__(self, path):
        if not isinstance(path,str) or not path.startswith('/dev/'):
            raise ValueError('explicit /dev/ tty path required')
        self.fd = None
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
        try:
            if not stat.S_ISCHR(os.fstat(fd).st_mode) or not os.isatty(fd):
                raise OSError('not a tty')
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            t = termios.tcgetattr(fd)
            t[0] = t[1] = t[3] = 0
            t[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            t[4] = t[5] = termios.B115200
            t[6][termios.VMIN] = 1; t[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, t)
            termios.tcflush(fd, termios.TCIOFLUSH)
            self.fd = fd
        except BaseException:
            os.close(fd); raise

    def read(self, limit):
        if self.fd is None: raise OSError('closed tty')
        try: data = os.read(self.fd, limit)
        except BlockingIOError: return b''
        if not data: raise OSError('tty EOF')
        return data

    def write(self, data):
        if self.fd is None: raise OSError('closed tty')
        return os.write(self.fd,data)

    def close(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            os.close(fd)
