import fnmatch
import os
from paramiko import SSHClient
from scp import SCPClient, SCPException
import argparse
from config import (host, user, remote_path_1)
"""
Script takes in the image as a command line argument.
SSH keys need to be configured on both remote and local machine in order for the script to work.
Host name, user name, and remote path are all imported from a config file. Update the information
there so that this script is portable.

"""

class remote_connection:

    def __init__(self, host, user, remote_path, file_name):
        self.host = host
        self.user = user
        self.remote_path = remote_path
        self.file_name = file_name
    def connect(self):
        """initiate the connection to the host machine""""
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
            print(error)

    def send_batch_images(self, image_directory):
        bulk_tranfer = [self.send_image(image) for image in image_directory]
        print(len(bulk_transfer) + " images have been transfered")


if __name__ == "__main__":

    #takes the image as a command line argument
    parser = argparse.ArgumentParser(description="the file that gets sent")
    parser.add_argument("image_directory", action="append")
    args = parser.parse_args()
    to_transfer = args.image_directory[0]

    image_files = []

    #Takes in a directory name from which a config file and jpeg files are extracted
    for name in os.listdir('test-file/'):
        if fnmatch.fnmatch(name, '*.txt'):
            config = 'test-file/' + name
        elif fnmatch.fnmatch(name, '*.jpeg'):
            image_files.append(name)
    #reads and splits the config file so that the variables can be used
    file_content = [line.split() for line in open(config, "r") if "#" not in line]
    flat_file_content = [i for flat in file_content for i in flat]

    username_index = flat_file_content.index("Username")+1
    userpath_index = flat_file_content.index("Userpath")+1
    remote_path_index = flat_file_content.index("Remote_path")+1
    #variables that can then be passed to the next function
    user = flat_file_content[username_index]
    host = flat_file_content[userpath_index]
    remote_path = flat_file_content[remote_path_index]

    sender = remote_connection(host, user, remote_path)
    sender.connect()
    if len(image_files) > 1:
        sender.send_batch_images(image_files)
    else:
        sender.send_image(image_files[0])
    sender.disconnect()
