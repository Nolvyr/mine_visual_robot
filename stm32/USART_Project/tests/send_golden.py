"""Run on Linux/S100. Explicit device required; sends one V3 golden, no ROS."""
import argparse
import binascii
import os
import select
import struct
import termios
import time


def golden():
    payload = struct.pack('<IHB', 0x12345678, 2, 13)
    for i in range(1, 14):
        payload += struct.pack('<BHIhhh', 1, 2, i, 100+i, -200-i, 300+i)
    frame = struct.pack('<2sBBBBIBH', b'\xaa\x55', 3, 0x10, 1, 0xf0,
                        0x01020304, 3, len(payload)) + payload
    frame += struct.pack('<H', binascii.crc_hqx(frame, 0xffff))
    assert len(frame) == 191 and frame[-2:] == bytes.fromhex('59 46')
    return frame


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', required=True, help='Confirmed non-console UART')
    p.add_argument('--read-debug', action='store_true', help='Read PA9 ASCII output for 2s')
    args = p.parse_args()
    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    old = None
    try:
        import fcntl
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        old = termios.tcgetattr(fd)
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        frame = golden()
        deadline, sent = time.monotonic()+2, 0
        while sent < len(frame):
            left = deadline-time.monotonic()
            if left <= 0 or not select.select([], [fd], [], left)[1]:
                raise TimeoutError('UART write deadline')
            try:
                n = os.write(fd, frame[sent:])
                if n == 0:
                    raise OSError('zero-byte write')
                sent += n
            except (BlockingIOError, InterruptedError):
                continue
        print('Driver accepted 191B; this alone is not an STM32 acknowledgement.')
        print(frame.hex(' ').upper())
        if args.read_debug:
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                if select.select([fd], [], [], max(0, deadline-time.monotonic()))[0]:
                    data = os.read(fd, 4096)
                    if data:
                        print(data.decode('ascii', errors='replace'), end='', flush=True)
    finally:
        try:
            if old is not None:
                termios.tcsetattr(fd, termios.TCSANOW, old)
        finally:
            os.close(fd)


if __name__ == '__main__':
    main()
