from __future__ import division

import serial
import threading
import struct
import numpy

API_ID_RX_PACKET_64 = 0x80
API_ID_TX_PACKET_16 = 0x01

class XBeeDriver(object):
    def __init__(self, devname, callback):
        self.callback = callback
        self.ser = serial.Serial(devname, 19200)
        self.thread = threading.Thread(target=self.run, name='XBeeParser')
        self.thread.setDaemon(True)
        self.thread.start()

    def send_packet(self, addr, data):
        packet_start = struct.pack(">BH", 0x7E, len(data)+5)
        packet_header = struct.pack(">BBHB", API_ID_TX_PACKET_16, 1, addr, 0)
        checksum = 0xFF - (sum(map(ord, packet_header + data)) & 0xFF)
        packet = packet_start + packet_header + data + chr(checksum)
        self.ser.write(packet)

    def run(self):
        while True:
            data = self.read_packet()
            if data == None:
                continue
            packet = self.parse_packet(data)
            if packet == None:
                continue
            self.callback(packet)

    def read_packet(self):
        self.ser.timeout = None
        while ord(self.ser.read()[0]) != 0x7E:
            pass

        self.ser.timeout = .1
        lengthbytes = self.ser.read(2)
        if len(lengthbytes) < 2:
            return None
        (length,) = struct.unpack(">H", lengthbytes)

        data = self.ser.read(length+1)
        if len(data) < length+1:
            return None

        checksum = sum(map(ord, data)) & 0xFF
        if checksum != 0xFF:
            return None

        return data[0:len(data)-1]

    def parse_packet(self, data):
        api_id = data[0]

        if ord(api_id) == API_ID_RX_PACKET_64:
            packet = dict(zip(['source', 'rssi', 'options'],
                            struct.unpack_from(">QBB", data[1:])))
            packet['rf_data'] = data[11:]
            return packet
        else:
            return None

status_fields = [
    ('roll', 'h', 10**4),
    ('pitch', 'h', 10**4),
    ('yaw', 'h', 10**4),
    ('roll_rate', 'h', 10**4),
    ('pitch_rate', 'h', 10**4),
    ('yaw_rate', 'h', 10**4),
    ('roll_bias', 'h', 10**4),
    ('pitch_bias', 'h', 10**4),
    ('yaw_bias', 'h', 10**4),
    ('roll_p', 'h', 10**4),
    ('pitch_p', 'h', 10**4),
    ('yaw_p', 'h', 10**4),
    ('roll_d', 'h', 10**4),
    ('pitch_d', 'h', 10**4),
    ('yaw_d', 'h', 10**4),
    ('gain_roll_p', 'h', 10**4),
    ('gain_pitch_p', 'h', 10**4),
    ('gain_yaw_p', 'h', 10**4),
    ('gain_roll_d', 'h', 10**4),
    ('gain_pitch_d', 'h', 10**4),
    ('gain_yaw_d', 'h', 10**4)]

gain_fields = [
    ('roll_p', 'h', 10**4),
    ('pitch_p', 'h', 10**4),
    ('yaw_p', 'h', 10**4),
    ('roll_d', 'h', 10**4),
    ('pitch_d', 'h', 10**4),
    ('yaw_d', 'h', 10**4)]

def parse_fields(fields, data):
    fmt = "<"+''.join(field[1] for field in fields)
    vals = struct.unpack(fmt, data)
    return { field[0]: val/field[2] for (field, val) in zip(fields, vals) }

def gen_fields(fields, data):
    fmt = "<"+''.join(field[1] for field in fields)
    return struct.pack(fmt, *(data[field[0]]*field[2] for field in fields))

class QuadState(object):
    def __init__(self, devname, callback):
        self.callback = callback
        self.xbee = XBeeDriver(devname, self.on_xbee_packet)
        self.data = {}

    def __getitem__(self, val):
        return self.data[val]

    def on_xbee_packet(self, packet):
        rf_data = packet['rf_data']
        if rf_data[0] == 's':
            self.data.update(parse_fields(status_fields, rf_data[1:]))
        self.callback()

    def send_gains(self, gains):
        self.xbee.send_packet(0xFFFF, 'g'+gen_fields(gain_fields, gains))
