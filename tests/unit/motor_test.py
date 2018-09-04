import os
import pty

from robot.basestation.motor import Motor
from robot.basestation.serialport import SerialPort


class TestMotor(object):
    # def __init__(self):
    #     self.mock_master, self.mock_slave = pty.openpty()
    #     self.mock_path = os.ttyname(self.mock_slave)

    def test_true(self):
        assert 1

    def test_write_mock(self):
        mock_master, mock_slave = pty.openpty()
        mock_path = os.ttyname(mock_slave)
        motor = Motor("1", serial_port=SerialPort(path=mock_path))
        motor.rotate(50)
        assert os.read(mock_master,
                       1000).decode() == str(motor.angle_position) + '\n'
        motor.rotate(50, 'ccw')
        assert os.read(mock_master,
                       1000).decode() == str(motor.angle_position) + '\n'


TestMotor().test_write_mock()