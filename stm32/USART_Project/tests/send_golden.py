"""Send the S100 V3 golden frame and optionally verify the STM32 binary ACK."""
import argparse
import binascii
import os
import select
import struct
import termios
import time

SESSION = 0x12345678
SEQUENCE = 0x01020304
MAP_BATCH = 0x10
ACK = 0x30


def golden():
    payload = struct.pack('<IHB', SESSION, 2, 13)
    payload += b''.join(struct.pack('<BHIhhh', 1, 2, i, 100+i, -200-i, 300+i)
                        for i in range(1, 14))
    frame = struct.pack('<2sBBBBIBH', b'\xaa\x55', 3, MAP_BATCH, 1, 0xf0,
                        SEQUENCE, 3, len(payload)) + payload
    frame += struct.pack('<H', binascii.crc_hqx(frame, 0xffff))
    assert len(frame) == 191 and frame[-2:] == bytes.fromhex('59 46')
    return frame


def read_exact(fd, size, timeout):
    result = bytearray()
    deadline = time.monotonic() + timeout
    while len(result) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
            raise TimeoutError(f'ACK timeout: received {len(result)}/{size} bytes')
        chunk = os.read(fd, size-len(result))
        if chunk:
            result.extend(chunk)
    return bytes(result)


def verify_ack(frame):
    if len(frame) != 21 or frame[:4] != bytes.fromhex('AA 55 03 30'):
        raise ValueError('invalid ACK header or length')
    if frame[4:6] != bytes.fromhex('F0 01') or frame[10:13] != bytes.fromhex('00 06 00'):
        raise ValueError('invalid ACK routing or payload length')
    if binascii.crc_hqx(frame[:-2], 0xffff) != struct.unpack('<H', frame[-2:])[0]:
        raise ValueError('ACK CRC mismatch')
    sequence = struct.unpack_from('<I', frame, 6)[0]
    session, message_type, status = struct.unpack_from('<IBB', frame, 13)
    if (sequence, session, message_type, status) != (SEQUENCE, SESSION, MAP_BATCH, 0):
        raise ValueError('ACK does not match the transmitted frame')
    print(f'[RxACK] session={session} seq={sequence} type=0x{message_type:02X} status=RECEIVED')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True, help='Confirmed non-console UART')
    parser.add_argument('--read-ack', '--read-debug', action='store_true',
                        help='Wait up to 2 seconds for the 21-byte STM32 ACK')
    args = parser.parse_args()
    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    try:
        import fcntl
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        original = termios.tcgetattr(fd)
        attributes = termios.tcgetattr(fd)
        attributes[0] = attributes[1] = attributes[3] = 0
        attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attributes[4] = attributes[5] = termios.B115200
        attributes[6][termios.VMIN] = attributes[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        frame = golden()
        deadline, sent = time.monotonic()+2, 0
        while sent < len(frame):
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
                raise TimeoutError('UART write deadline')
            try:
                count = os.write(fd, frame[sent:])
                if count == 0:
                    raise OSError('zero-byte write')
                sent += count
            except (BlockingIOError, InterruptedError):
                continue
        print('Driver accepted 191B; waiting for ACK is required to confirm STM32 reception.')
        if args.read_ack:
            verify_ack(read_exact(fd, 21, 2.0))
    finally:
        try:
            if original is not None:
                termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            os.close(fd)


if __name__ == '__main__':
    main()