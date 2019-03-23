# this class will be used to test if client can connect to the raover
import os
import socket


class ClientConnection:
    com_port = 5000


    def __init__(self, base_port, rover_ip, rover_port, status=False):
        self.status = status
        self.base_ip = self.get_base_ip()
        self.set_rover_ip(rover_ip)

    def set_status(self, status):
        self.status = status

    def get_status(self):
        return self.status

    def set_base_ip(self, base_ip):
        self.base_ip = base_ip

    def get_base_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        base_ip = s.getsockname()[0]
        print("base_ip: " + base_ip)
        s.close()

        return base_ip

    def set_base_port(self, base_port):
        self.base_port = base_port

    def get_base_port(self):
        return self.base_port

    def set_rover_ip(self, rover_ip):
        self.rover_ip = rover_ip

    def get_rover_ip(self):
        return self.rover_ip

    def set_rover_port(self, rover_port):
        self.rover_port = rover_port

    def get_rover_port(self):
        return self.rover_port

    def set_serial_port(self, serial_port):
        self.serial_port = serial_port

    def get_serial_port(self):
        return self.serial_port

    # the ping test method is to check if a connection can be returned from to rover
    def ping_test(self):
        """
        Sends one ping request to given IP address

        :return: boolean for ping success status
        """

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a TCP/IP socket
        server_ip = input("Enter server IP: ")
        rep = os.system("ping " + server_ip + " -c 1")
        if rep == 0:
            # the server is up so return true
            self.status = True
            print("connection established")
            return self.status
        else:
            # the server is down so return false
            self.status = False
            print("no response from server")
            return self.status

    def send_drive_cmd(self):
        """
        method to send commands to the rover

        """

    def get_motor_currents(self):
        """

        :return: array of each motor's currents
        """

    def get_motor_positions(self):
        """

        :return: array of motor positions
        """

    def get_stream(self):
        """

        :return: video stream from camera
        """

    def get_logs(self):
        """
        read the logs from the rover and return them
        :return:
        """
