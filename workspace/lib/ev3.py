import unittest
import os
import configparser
import serial
import time
import socket
import queue
from datetime import datetime
from enum import Enum

class MsgId(Enum):
    # タスク制御等
    ENABLE_WATCHDOG_TASK = 10
    # モーター
    MOTOR_CONFIG = 50
    MOTOR_STEER = 51
    MOTOR_GET_COUNTS = 52
    MOTOTR_GET_POWER = 53
    MOTOR_RESET_COUNTS = 54
    MOTOR_ROTATE = 55
    MOTOR_SET_POWER = 56
    MOTOR_STOP = 57
    # センサ共通
    SENSOR_CONFIG = 100
    # タッチセンサ
    TOUCH_SENSOR_IS_PRESSED = 110
    # カラーセンサ
    COLOR_SENSOR_GET_REFLECT = 120
    COLOR_SENSOR_GET_AMBIENT = 121
    COLOR_SENSOR_GET_COLOR = 122
    COLOR_SENSOR_GET_RGB_RAW = 123
    # ジャイロセンサ
    GYRO_SENSOR_RESET = 130
    GYRO_SENSOR_GET_ANGLE = 131
    GYRO_SENSOR_GET_RATE = 132
    # 超音波センサ
    ULTRASONIC_SENSOR_GET_DISTANCE = 140
    ULTRASONIC_SENSOR_LISTEN = 141
    # 赤外線センサ
    INFRARED_SENSOR_GET_DISTANCE = 150
    INFRARED_SENSOR_GET_REMOTE = 151
    INFRARED_SENSOR_SEEK = 152
    # EV3本体
    LCD_DRAW_STRING = 200
    BUTTON_IS_PRESSED = 210


class ColorId(Enum):
    COLOR_NONE = 0
    COLOR_BLACK = 1
    COLOR_BLUE = 2
    COLOR_GREEN = 3
    COLOR_YELLOW = 4
    COLOR_RED = 5
    COLOR_WHITE = 6
    COLOR_BROWN = 7
    TNUM_COLOR = 8


class BaseCommunicator():
    def write(self, items):
        raise NotImplementedError()

    def read(self):
        raise NotImplementedError()

    def close(self):
        raise NotImplementedError()


class RealCommunicator(BaseCommunicator):
    def __init__(self, portname):
        self.ser = serial.Serial(portname, 115200, timeout=None)
        self.ser.reset_input_buffer()
        self.last_written_time = datetime.now().timestamp()

    def _encode_byte(self, item):
        assert(0 <= item and item < 256)
        q1, mod1 = divmod(item, 16)
        q2, mod2 = divmod(q1, 16)
        assert(q2 == 0)
        return (mod2, mod1)

    def write(self, items):
        for item in items:
            for b in self._encode_byte(item):
                # Wait for transmission interval
                current_time = datetime.now().timestamp()
                while (current_time - self.last_written_time) < 0.003:
                    current_time = datetime.now().timestamp()
                self.ser.write(bytes([b]))
                self.last_written_time = datetime.now().timestamp()

    def read(self):
        # Clear input buffer until receive b'\xfe'
        while True:
            r = self.ser.read(1)
            if r == b'\xfe':
                break
        r = self.ser.read(1)
        return int(r[0])

    def close(self):
        self.ser.close()


class SimCommunicator(BaseCommunicator):
    class Receiver:
        def __init__(self, host, port):
            self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server.bind((host, port))

        def receive(self, bufsize=1024):
            rcvmsg, _ = self.server.recvfrom(bufsize)
            return rcvmsg

        def close(self):
            self.server.close()

    class Sender:
        def __init__(self, host, port):
            self.address = (host, port)
            self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def send(self, data):
            self.client.sendto(data, self.address)

        def close(self):
            self.client.close()

    def __init__(self, rpi_address, rpi_port, sim_address, sim_port):
        self.que = queue.Queue()
        self.recver = self.Receiver(rpi_address, rpi_port)
        self.sender = self.Sender(sim_address, sim_port)
        self.last_sent_time = datetime.now().timestamp()

    def __del__(self):
        self.close()

    def write(self, items):
        for item in items:
            current_time = datetime.now().timestamp()
            while (current_time - self.last_sent_time) < 0.006:
                current_time = datetime.now().timestamp()
            self.sender.send(bytes([item]))
            self.last_sent_time = datetime.now().timestamp()

    def _receive(self, bufsize=1024):
        rsvmsg = self.recver.receive(bufsize)
        return list(map(int, rsvmsg))

    def read(self):
        if self.que.empty():
            data = self._receive()
            for x in data:
                self.que.put(x)
        return self.que.get()

    def close(self):
        self.recver.close()
        self.sender.close()


class EV3():
    # sensor_port_t
    PORT_1 = 0
    PORT_2 = 1
    PORT_3 = 2
    PORT_4 = 3

    # sensor_type_t
    NONE_SENSOR = 0
    ULTRASONIC_SENSOR = 1
    GYRO_SENSOR = 2
    TOUCH_SENSOR = 3
    COLOR_SENSOR = 4
    INFRARED_SENSOR = 5
    HT_NXT_ACCEL_SENSOR = 6
    NXT_TEMP_SENSOR = 7
    TNUM_SENSOR_TYPE = 8

    # colorid_t
    COLOR_NONE = 0
    COLOR_BLACK = 1
    COLOR_BLUE = 2
    COLOR_GREEN = 3
    COLOR_YELLOW = 4
    COLOR_RED = 5
    COLOR_WHITE = 6
    COLOR_BROWN = 7
    TNUM_COLOR = 8

    # motor_port_t
    PORT_A = 0
    PORT_B = 1
    PORT_C = 2
    PORT_D = 3
    TNUM_MOTOR_PORT = 4

    # motor_type_t
    NONE_MOTOR = 0
    MEDIUM_MOTOR = 1
    LARGE_MOTOR = 2
    UNREGULATED_MOTOR = 3
    TNUM_MOTOR_TYPE = 4

    # button_t
    LEFT_BUTTON = 0
    RIGHT_BUTTON = 1
    UP_BUTTON = 2
    DOWN_BUTTON = 3
    ENTER_BUTTON = 4
    BACK_BUTTON = 5
    TNUM_BUTTON = 6

    def __init__(self):
        inifile = configparser.SafeConfigParser()
        inifile.read('./config.ini')
        env = inifile.get('environment', 'env')
        if env == 'real_ev3':
            portname = inifile.get('real_ev3', 'port')
            self.com = RealCommunicator(portname)
        elif env == 'simulated_ev3':
            rpi_address = inifile.get('simulated_ev3', 'rpi_address')
            rpi_port = inifile.getint('simulated_ev3', 'rpi_port')
            sim_address = inifile.get('simulated_ev3', 'sim_address')
            sim_port = inifile.getint('simulated_ev3', 'sim_port')
            self.com = SimCommunicator(rpi_address, rpi_port,
                                       sim_address, sim_port)
        else:
            raise NotImplementedError()
        for i in range(7):
            self.lcd_clear_line(i)

    def __del__(self):
        self.close()
        
    def close(self):
        self.com.close()

    def _write(self, items):
        self.com.write(items)

    def _read(self):
        return self.com.read()

    def _send_header(self, cmd_id):
        self._write([255, cmd_id])

    def motor_config(self, motor_port, motor_type):
        self._send_header(MsgId.MOTOR_CONFIG.value)
        self._write([motor_port, motor_type])

    # タスク制御等
    def enable_watchdog_task(self):
        self._send_header(MsgId.ENABLE_WATCHDOG_TASK.value)

    # モーター
    def motor_steer(self, left_motor_port, right_motor_port, drive, steer):
        self._send_header(MsgId.MOTOR_STEER.value)
        drive = int(drive) + 100
        steer = int(steer) + 100
        drive = min(max(drive, 0), 200)
        steer = min(max(steer, 0), 200)
        self._write([left_motor_port, right_motor_port, drive, steer])

    # センサ共通
    def sensor_config(self, sensor_port, sensor_type):
        self._send_header(MsgId.SENSOR_CONFIG.value)
        self._write([sensor_port, sensor_type])

    # タッチセンサ
    def touch_sensor_is_pressed(self, touch_sensor_port):
        self._send_header(MsgId.TOUCH_SENSOR_IS_PRESSED.value)
        self._write([touch_sensor_port])
        touch = self._read()
        assert(touch == 1 or touch == 0)
        return True if touch == 1 else False

    # カラーセンサ
    def color_sensor_get_reflect(self, color_sensor_port):
        self._send_header(MsgId.COLOR_SENSOR_GET_REFLECT.value)
        self._write([color_sensor_port])
        color = self._read()
        assert(color <= 100)
        return color

    def color_sensor_get_ambient(self, color_sensor_port):
        self._send_header(MsgId.COLOR_SENSOR_GET_AMBIENT.value)
        self._write([color_sensor_port])
        ambient = self._read()
        assert(ambient <= 100)
        return ambient

    def color_sensor_get_color(self, color_sensor_port):
        self._send_header(MsgId.COLOR_SENSOR_GET_COLOR.value)
        self._write([color_sensor_port])
        color = self._read()
        return color

    def color_sensor_get_rgb_raw(self, color_sensor_port):
        self._send_header(MsgId.COLOR_SENSOR_GET_RGB_RAW.value)
        self._write([color_sensor_port])
        rgb = []
        for i in range(3):
            data1 = self._read()
            data2 = self._read()
            rgb.append(data1 << 8 | data2)
        return rgb

    # ジャイロセンサ
    def gyro_sensor_reset(self, gyro):
        self._send_header(MsgId.GYRO_SENSOR_RESET.value)
        self._write([gyro])

    def gyro_sensor_get_angle(self, gyro):
        self._send_header(MsgId.GYRO_SENSOR_GET_ANGLE.value)
        self._write([gyro])
        angle1 = self._read()
        angle2 = self._read()
        angle = angle1 << 8 | angle2
        return int.from_bytes(angle.to_bytes(2, byteorder = "big"), byteorder='big', signed=True)

    def gyro_sensor_get_rate(self, gyro):
        self._send_header(MsgId.GYRO_SENSOR_GET_RATE.value)
        self._write([gyro])
        rate1 = self._read()
        rate2 = self._read()
        rate = rate1 << 8 |  rate2
        return int.from_bytes(rate.to_bytes(2, byteorder = "big"), byteorder='big', signed=True)

    # 超音波センサ
    def ultrasonic_sensor_get_distance(self, ultrasonic):
        self._send_header(MsgId.ULTRASONIC_SENSOR_GET_DISTANCE.value)
        self._write([ultrasonic])
        distance1 = self._read()
        distance2 = self._read()
        distance = distance1 << 8 | distance2
        return distance

    def ultrasonic_sensor_listen(self, ultrasonic):
        self._send_header(MsgId.ULTRASONIC_SENSOR_LISTEN.value)
        self._write([ultrasonic])
        listen = self._read()
        assert(listen == 1 or listen == 0)
        return True if listen == 1 else False

    # 赤外線センサ
    def infrared_sensor_get_distance(self, infrared):
        self._send_header(MsgId.INFRARED_SENSOR_GET_DISTANCE.value)
        self._write([infrared])
        distance = self._read()
        return distance

    def infrared_sensor_get_remote(self, infrared):
        self._send_header(MsgId.INFRARED_SENSOR_GET_REMOTE.value)
        self._write([infrared])
        channel = []
        for i in range(4):
            channel.append(self._read())
        return channel

    def infrared_sensor_seek(self, infrared):
        self._send_header(MsgId.INFRARED_SENSOR_SEEK.value)
        self._write([infrared])
        heading = []
        for i in range(4):
            channel.append(self._read())
        distance = []
        for i in range(4):
            distance.append(self._read())
        return heading, distance

    # EV3本体
    def lcd_draw_string(self, string, line):
        self._send_header(MsgId.LCD_DRAW_STRING.value)
        self._write([line])
        if len(string) < 20:
            string = string + ' ' * (20 - len(string))
        items = [ord(x) for x in string]
        items.append(0)
        self._write(items)

    def lcd_clear_line(self, line):
        self.lcd_draw_string(' ' * 20, line)

    def button_is_pressed(self, button):
        self._send_header(MsgId.BUTTON_IS_PRESSED.value)
        self._write([button])
        state = self._read()
        assert(state == 1 or state == 0)
        return True if state == 1 else False



class TestEV3(unittest.TestCase):
    def test_write(self):
        ev3 = EV3()
        for i in [244, 245, 246]:
            ev3._write([255, i])
            time.sleep(2)

    def test_touch_sensor(self):
        port = EV3.PORT_2
        ev3 = EV3()
        ev3.sensor_config(port, EV3.TOUCH_SENSOR)
        for i in range(5):
            touch = ev3.touch_sensor_is_pressed(port)
            print("touch = {0}".format(touch))
            time.sleep(2)

    def test_color_sensor(self):
        port = EV3.PORT_3
        ev3 = EV3()
        ev3.sensor_config(port, EV3.COLOR_SENSOR)
        for i in range(5):
            color = ev3.color_sensor_get_reflect(port)
            print("color = {0}".format(color))
            time.sleep(2)

    def test_motors(self):
        lport = EV3.PORT_B
        rport = EV3.PORT_C
        ev3 = EV3()
        ev3.motor_config(lport, EV3.LARGE_MOTOR)
        ev3.motor_config(rport, EV3.LARGE_MOTOR)

        for i in range(-100, 100):
            print("send drive = 10, steer = {}".format(i))
            ev3.motor_steer(lport, rport, 10, i)
            time.sleep(0.1)


if __name__ == '__main__':
    unittest.main()
