"""
This script connects to the rover using ssh and then transfers the contents of
a directory back to the base station. A new directory is created on the base station.

User details, rover ip, image directory, and destination directory are all updated via
a dictonary in transferconfig.py

The ssh connection depends on the user having an rsa key pair between the base station
and the rover.
"""
import os
import os.path
from os import path
import subprocess
from subprocess import Popen, PIPE
from paramiko import SSHClient, RSAKey
from paramiko.auth_handler import SSHException
from scp import SCPClient, SCPException
import transferconfig as cfg

class RemoteConnection:

    def __init__(self, rover_ip, user, rover_file_path, key_path):
        self.rover_ip = rover_ip
        self.user = user
        self.rover_file_path = rover_file_path
        self.key_path = key_path
        self.ssh_key = None

    def generate_rsa_key(self):
        """generates a defauls rsa key in the .ssh folder. Will push rsa key to the rover.
        Will only run if a key is not found with name specified in transferconfig.py. Default name
        is id_rsa
        alter name in transferconfig.py if you want to set up a discrete key for this usage."""
        key_name = "ToDo"
        subprocess.run(f'ssh-keygen -f {self.key_path} -t rsa', shell=True)

    def check_ssh_keys(self):
        """checks if system has an ssh key for the server"""
        path = os.path.expanduser(self.key_path)
        #print(path)
        try:
            RSAKey.from_private_key_file(path, password='Mackie66')
            print("key found")
        except (FileNotFoundError, SSHException):
            print('key is not found')
            self.generate_rsa_key()

    def connect(self):
        """initiate the connection to the host machine"""
        self.client = SSHClient()
        self.client.load_system_host_keys()
        self.client.connect(hostname=self.rover_ip, username=self.user)
        self.scp = SCPClient(self.client.get_transport())

    def disconnect(self):
        self.client.close()
        self.scp.close()

    def get_images(self, basestation_directory):
        rover_file_path = self.rover_file_path
        try:
            self.scp.get(rover_file_path, basestation_directory, recursive=True)
            print("Images transfered from rover to base station")
        except SCPException as error:
            print(error + "Directory was not found")





if __name__ == "__main__":

    image_retriver = RemoteConnection(cfg.user['rover_ip'], cfg.user['username'],
                                      cfg.user['rover_image_dir'], cfg.user['key_path'])
    beans = image_retriver.check_ssh_keys()
   # if not beans:
    #    image_retriver.generate_rsa_key()
     #   print("all done")
   # else:
    #    print("failed")
    #image_retriver.connect()
    #image_retriver.get_images(cfg.user['basestation_dir'])
    #image_retriver.disconnect()
