#!/usr/bin/env python
"""
This script connects to the rover using ssh and then transfers the contents of
a directory back to the base station. A new directory is created on the base station.

User details, rover ip, image directory, and destination directory are all updated via
a dictonary in transferconfig.py

"""
import os
import os.path
import subprocess
import sys
import traceback
from paramiko import SSHClient, AutoAddPolicy, RSAKey
from paramiko.auth_handler import SSHException
from scp import SCPClient, SCPException
import transferconfig as cfg


class RemoteConnection:

    def __init__(self, rover_ip, user, rover_file_path, key_path, rsa_password):
        self.rover_ip = rover_ip
        self.user = user
        self.rover_file_path = rover_file_path
        self.rsa_password = rsa_password
        self.key_path = key_path
        self.ssh_key = None
        self.scp = None

    def check_ssh_keys(self):
        """checks if system has an RSA key for the server. if key is not found
        will generate a new key at the location specified in the config file"""
        path = os.path.expanduser(self.key_path)
        try:
            RSAKey.from_private_key_file(path, password=self.rsa_password)
            print("key found")
        except FileNotFoundError:
            print('key is not found')
            subprocess.run(f'ssh-keygen -f {self.key_path} -t rsa', shell=True)
            print(f'RSA key generated at {self.key_path}')
        except SSHException:
            print(
                'There is a problem with the key, double check that your encription' +
                ' password is correct'
            )
            sys.exit()

    def connect(self):
        """initiate the connection to the host machine. If the rsa key has not been
        pushed to rover, function will handle it"""
        try:
            self.client = SSHClient()
            self.client.load_system_host_keys()
            self.client.set_missing_host_key_policy(AutoAddPolicy())
            self.client.connect(
                hostname=self.rover_ip, username=self.user, password=self.rsa_password
            )
            self.scp = SCPClient(self.client.get_transport())
            print(f'sucessfully connected to {self.rover_file_path}')
        except SSHException:
            subprocess.run(
                f'ssh-copy-id -i {self.key_path} {self.user}@{self.rover_ip}',
                shell=True
            )
            subprocess.run('ssh-add')

    def disconnect(self):
        self.client.close()
        self.scp.close()

    def get_images(self, basestation_directory, file_subdirectory):
        rover_file_path = self.rover_file_path

        try:
            self.scp.get(
                rover_file_path,
                basestation_directory + file_subdirectory,
                recursive=True
            )
            print('Images transfered from rover to base station')
            print('From ' + rover_file_path + ' to ' + basestation_directory)
        except SCPException:
            traceback.print_exc()
        except Exception:
            traceback.print_exc()


def check_payload_exists(basestation_directory):
    if os.path.isdir(basestation_directory):
        print(
            f'file {basestation_directory} exists and the files from the rover will be copied there \n'
            + 'press enter to continue or n to abort and edit transferconfig.py'
        )
        exit_condition = input()
        if exit_condition.lower() == 'n':
            sys.exit()
    else:
        print(
            f'{basestation_directory} does not exist and will be created at that location'
        )
        subprocess.run(f'mkdir {basestation_directory}', shell=True)


def check_payload_subdir(basestation_directory, payload_sub):
    if os.path.exists(basestation_directory + payload_sub):
        #two ideas here:
        #print(f'{basestation_directory}{payload_sub} already exists, please change the name in the config file')
        #sys.exit()
        #or:
        print(
            f'{basestation_directory}{payload_sub} already exists, please enter a new subdirectory name that is not in this list'
        )
        thing = os.listdir(basestation_directory)
        print(thing)
        new_name = input()
        return new_name
    else:
        return f'{payload_sub}'


if __name__ == "__main__":
    check_payload_exists(cfg.user['basestation_dir'])
    file_dir = check_payload_subdir(
        cfg.user['basestation_dir'], cfg.user['payload_file_subdir']
    )
    image_retriver = RemoteConnection(
        cfg.user['rover_ip'], cfg.user['rover_user'], cfg.user['rover_image_dir'],
        cfg.user['key_path'], cfg.user['rsa_password']
    )
    image_retriver.check_ssh_keys()
    image_retriver.connect()
    image_retriver.get_images(cfg.user['basestation_dir'], file_dir)
    image_retriver.disconnect()
