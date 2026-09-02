"""Two real ROS processes, isolated domain; no camera or serial device required."""
import os
import signal
import subprocess
import sys
import tempfile
import pty
import select
import struct
import binascii

env = os.environ.copy()
env['ROS_DOMAIN_ID'] = '189'
env['ROS_LOCALHOST_ONLY'] = '1'
env['LD_LIBRARY_PATH'] = os.path.dirname(os.path.abspath(sys.argv[1])) + os.pathsep + env.get('LD_LIBRARY_PATH', '')
with tempfile.TemporaryDirectory(prefix='gateway-ros-test-') as logs:
    env['ROS_LOG_DIR'] = logs
    with tempfile.TemporaryFile(mode='w+') as output:
        gateway_args = [sys.argv[1]]
        producer_args = [sys.argv[2]]
        master = slave = None
        mode = sys.argv[3] if len(sys.argv) > 3 else 'simulated'
        if mode == 'uart':
            master, slave = pty.openpty()
            gateway_args += ['--ros-args', '-p', 'transport_mode:=uart', '-p',
                             'serial_device:=' + os.ttyname(slave)]
        elif mode == 'unavailable':
            gateway_args += ['--ros-args', '-p', 'transport_mode:=uart', '-p',
                             'serial_device:=/dev/nonexistent-codex-uart']
            producer_args += ['--expect-transport-failure']
        gateway = subprocess.Popen(gateway_args, env=env, stdout=output, stderr=subprocess.STDOUT)
        try:
            producer = subprocess.run(producer_args, env=env, stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, text=True, timeout=40)
            print(producer.stdout)
            status = producer.returncode
            if mode == 'uart' and status == 0:
                payload = struct.pack('<IHB', 1234, 2, 13) + b''.join(
                    struct.pack('<BHIhhh', 1, 2, i, 100+i, -200-i, 300+i) for i in range(1,14))
                expected = bytes.fromhex('AA 55 03 10 01 F0') + struct.pack('<IBH',20,3,len(payload)) + payload
                expected += struct.pack('<H', binascii.crc_hqx(expected, 0xFFFF))
                received = b''
                while len(received) < len(expected):
                    if not select.select([master], [], [], 2)[0]:
                        raise RuntimeError('PTY receive timeout')
                    received += os.read(master, len(expected)-len(received))
                if received != expected:
                    raise RuntimeError('Gateway UART bytes differ from independent V3 golden')
                print('[UART_PTY] full independent golden match: 191B')
        finally:
            gateway.send_signal(signal.SIGINT)
            try:
                gateway.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gateway.kill()
                gateway.wait()
            output.seek(0)
            print(output.read())
            if master is not None:
                os.close(master)
                os.close(slave)
        sys.exit(status)
