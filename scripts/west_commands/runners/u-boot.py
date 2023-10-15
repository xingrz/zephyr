# Copyright (c) 2023 Chen Xingyu <hi@xingrz.me>
# SPDX-License-Identifier: Apache-2.0

'''Runner for loading Zephyr with U-Boot loady protocol'''

import struct
from serial import Serial
from os.path import basename
from progress.bar import Bar

from runners.core import ZephyrBinaryRunner, RunnerCaps, BuildConfiguration

Y_SOH = b'\x01'
Y_STX = b'\x02'
Y_EOT = b'\x04'
Y_ACK = b'\x06'

def loady(ser, name, data):
    def _soh(seq, name='', size=''):
        payload = f'{name}\x00{size}\x00'.encode().ljust(128, b'\x00')
        pkt = bytearray(Y_SOH)
        pkt += struct.pack('<H', palindrome(seq))
        pkt += payload
        pkt += struct.pack('>H', crc16(payload))
        return pkt

    def _stx(seq, data):
        payload = data.ljust(1024, b'\x00')
        pkt = bytearray(Y_STX)
        pkt += struct.pack('<H', palindrome(seq))
        pkt += payload
        pkt += struct.pack('>H', crc16(payload))
        return pkt

    def _read(expected):
        while True:
            n = ser.read(1024)
            if len(n) == 0:
                continue
            elif len(n) > len(expected):
                print(n.decode('ascii', errors='replace').strip())
                return False
            return n == expected

    ser.write(_soh(0, name, len(data)))
    if not _read(Y_ACK):
        return False

    with Bar(max=len(data)) as bar:
        for i in range(0, len(data), 1024):
            idx = i // 1024 + 1
            block = data[i:i + 1024]
            ser.write(_stx(idx, block))
            if not _read(Y_ACK):
                return False
            bar.next(len(block))

    ser.write(Y_EOT)
    if not _read(Y_ACK + Y_ACK + b'C'):
        return False

    ser.write(_soh(0))
    if not _read(Y_ACK):
        return False

    return True

def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF

def palindrome(n):
    return ((~n & 0xff) << 8) | (n & 0xff)

def run_cmd(ser, cmd, debug=False):
    while True:
        n = ser.read(1024)
        if len(n) == 0:
            continue

        if debug:
            print(n.decode('ascii', errors='replace').strip())

        if b'U-Boot>' in n:
            ser.write(f'{cmd}\n'.encode())
            continue

        if cmd.encode() in n:
            return

class UBootBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for U-Boot'''

    def __init__(self, cfg, port, load_address, boot, debug):
        super().__init__(cfg)
        self.port = port
        self.app_bin = cfg.bin_file
        self.app_img = f'{self.app_bin}.uimg'
        self.load_address = load_address
        self.boot = boot
        self.debug = debug

    @classmethod
    def name(cls):
        return 'u-boot'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'})

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument('--port', required=True,
                            help='serial port to flash')
        parser.add_argument('--boot', action='store_true',
                            help='boot after flashing')
        parser.add_argument('--debug', action='store_true',
                            help='print device logs during flashing')

    @classmethod
    def do_create(cls, cfg, args):
        build_conf = BuildConfiguration(cfg.build_dir)
        load_address = build_conf['CONFIG_SRAM_BASE_ADDRESS']

        return UBootBinaryRunner(cfg,
                                 args.port,
                                 load_address,
                                 args.boot,
                                 args.debug)

    def do_run(self, command, **kwargs):
        with open(self.app_bin, 'rb') as f:
            app = f.read()

        ser = Serial(self.port, 115200, timeout=0.05)

        print('-- Waiting for device reset...')

        while True:
            n = ser.read(1024)
            if len(n) == 0:
                continue

            if self.debug:
                print(n.decode('ascii', errors='replace').strip())

            if b'Hit any key to stop autoboot:' in n:
                ser.write(b'\n')
                break

        print('-- Loading...')
        run_cmd(ser, f'loady {self.load_address} 115200', self.debug)
        loady(ser, basename(self.app_img), app)
        run_cmd(ser, f'fatwrite mmc 0:1 {self.load_address} zephyr.bin {len(app)}', self.debug)

        if self.boot:
            print('-- Booting...')
            run_cmd(ser, f'go {self.load_address}', self.debug)
        else:
            print('-- Done')
