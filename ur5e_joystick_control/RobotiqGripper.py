#!/usr/bin/env python
import socket
import time
import struct
import threading

# Class for robotiq controller
class RobotiqHand():
    def __init__(self):
        self.so = None
        self._cont = False
        self._sem = None
        self._heartbeat_th = None
        self._max_position = 255
        self._min_position = 0

    def _heartbeat_worker(self):
        while self._cont:
            self.status()
            time.sleep(0.5)

    def connect(self, ip, port):
        self.so = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.so.connect((ip, port))
        self._cont = True
        self._sem = threading.Semaphore(1)
        self._heartbeat_th = threading.Thread(target = self._heartbeat_worker)
        self._heartbeat_th.start()

    def disconnect(self):
        self._cont = False
        self._heartbeat_th.join()
        self._heartbeat_th = None
        self.so.close()
        self.so = None
        self._sem = None

    def _calc_crc(self, command):
        crc_registor = 0xFFFF
        for data_byte in command:
            tmp = crc_registor ^ data_byte
            for _ in range(8):
                if(tmp & 1 == 1):
                    tmp = tmp >> 1
                    tmp = 0xA001 ^ tmp
                else:
                    tmp = tmp >> 1
            crc_registor = tmp
        crc = bytearray(struct.pack('<H', crc_registor))
        return crc

    def send_command(self, command):
        with self._sem:
            crc = self._calc_crc(command)
            data = command + crc
            self.so.sendall(data)
            time.sleep(0.001)
            data = self.so.recv(1024)
        return bytearray(data)

    def status(self):
        command = bytearray(b'\x09\x03\x07\xD0\x00\x03')
        return self.send_command(command)

    def reset(self):
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00')
        return self.send_command(command)

    def activate(self):
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00')
        return self.send_command(command)

    def wait_activate_complete(self):
        while True:
            data = self.status()
            if data[5] != 0x00:
                return data[3]
            if data[3] == 0x31 and data[7] < 4:
                return data[3]

    def adjust(self):
        self.move(255, 64, 1)
        (status, position, force) = self.wait_move_complete()
        self._max_position = position
        self.move(0, 64, 1)
        (status, position, force) = self.wait_move_complete()
        self._min_position = position

    def get_position_mm(self, position):
        if position > self._max_position:
            position = self._max_position
        elif position < self._min_position:
            position = self._min_position
        position_mm = 85.0 * (self._max_position - position) / (self._max_position - self._min_position)
        #print 'max=%d, min=%d, pos=%d pos_mm=%.1f' % (self._max_position, self._min_position, position, position_mm)
        return position_mm

    def get_force_mA(self, force):
        return 10.0 * force

    # position: 0x00...open, 0xff...close
    # speed: 0x00...minimum, 0xff...maximum
    # force: 0x00...minimum, 0xff...maximum
    def move(self, position, speed, force):
        #print('move hand')
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\x00\x00')
        command[10] = position
        command[11] = speed
        command[12] = force
        return self.send_command(command)

    # result: (status, position, force)
    def wait_move_complete(self):
        while True:
            data = self.status()
            if data[5] != 0x00:
                return (-1, data[7], data[8])
            if data[3] == 0x79:
                return (2, data[7], data[8])
            if data[3] == 0xb9:
                return (1, data[7], data[8])
            if data[3] == 0xf9:
                return (0, data[7], data[8])

    def query_complete(self):
        data = self.status()
        if data[5] != 0x00:
            return -1
        if data[3] == 0x79:
            return 2
        if data[3] == 0xb9:
            return 1
        if data[3] == 0xf9:
            return 0
        return 7
