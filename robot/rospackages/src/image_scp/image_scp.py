from paramiko import SSHClient
from scp import SCPClient, SCPException
import argparse
from config import (host, user, remote_path_1)
"""
Script takes in the image as a command line argument.
SSH keys need to be configured on both remote and local machine in order for the script to work.
Host name, user nmae, and remote path are all imported from a config file. Update the information there 
so that this script is portable .

"""
class remote_connection:
    def __init__(self, host, user, remote_path, file_name):
        self.host = host
        self.user = user
        self.remote_path = remote_path
        self.file_name = file_name
#initiate the connection to the host machine
    def connect(self):
        self.client = SSHClient()
        self.client.load_system_host_keys()
        self.client.connect(hostname=self.host, username=self.user)
        self.scp = SCPClient(self.client.get_transport())

    def disconnect(self):
        self.client.close()
        self.scp.close()

    def send_image(self, image):
        try:
            self.scp.put(image, recursive=True, remote_path=self.remote_path)
        except SCPException as error:
            raise error

if __name__ == "__main__":
    #takes the image as a command line argument
    parser = argparse.ArgumentParser(description="the file that gets sent")
    parser.add_argument("image_file", action="append")
    args = parser.parse_args()
    to_transfer = args.image_file[0]

    sender = remote_connection(host, user, remote_path_1, to_transfer)
    sender.connect()
    sender.send_image(sender.file_name)
    sender.disconnect()
