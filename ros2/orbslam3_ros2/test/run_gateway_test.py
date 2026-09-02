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
import time

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
        if mode in ('uart', 'uart_bad_ack', 'uart_no_ack'):
            master, slave = pty.openpty()
            ack_timeout='500' if mode=='uart' else '100'
            gateway_args += ['--ros-args', '-p', 'transport_mode:=uart', '-p',
                             'serial_device:=' + os.ttyname(slave), '-p',
                             'ack_timeout_ms:=' + ack_timeout]
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
            if mode in ('uart', 'uart_bad_ack', 'uart_no_ack') and status == 0:
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
                if mode!='uart_no_ack':
                    ack_session=999 if mode=='uart_bad_ack' else 1234
                    ack_payload=struct.pack('<IBB',ack_session,0x10,0)
                    ack=bytes.fromhex('AA 55 03 30 F0 01')+struct.pack('<IBH',20,0,len(ack_payload))+ack_payload
                    ack+=struct.pack('<H',binascii.crc_hqx(ack,0xFFFF))
                    if len(ack)!=21: raise RuntimeError('ACK golden length mismatch')
                    os.write(master,ack[:5]);os.write(master,ack[5:])
                time.sleep(0.25)
        finally:
            gateway.send_signal(signal.SIGINT)
            try:
                gateway.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gateway.kill()
                gateway.wait()
            output.seek(0)
            gateway_output=output.read();print(gateway_output)
            if mode=='uart' and status==0 and '[RxACK] session=1234 seq=20 type=0x10 status=RECEIVED peer=STM32' not in gateway_output:
                status=1;print('missing matched STM32 ACK log')
            if mode=='uart_bad_ack' and status==0 and ('[RxACK] unmatched' not in gateway_output or '[ACK_TIMEOUT] session=1234 seq=20 type=0x10' not in gateway_output):
                status=1;print('mismatched ACK was not rejected and expired')
            if mode=='uart_no_ack' and status==0 and '[ACK_TIMEOUT] session=1234 seq=20 type=0x10' not in gateway_output:
                status=1;print('missing ACK timeout log')
            if master is not None:
                os.close(master)
                os.close(slave)
        sys.exit(status)
