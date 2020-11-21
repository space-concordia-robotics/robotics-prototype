#! usr/bin/env python3
"""
Default configuration is set to work with the rover

Update user credentials here for host username, the host ip address,
diectory where the images are located on the rover, and the destination directory on the base
station.

basestation_dir will be the name of the destination directory for copying over the contents from the rover_image_dir.

key_path is the path to your rsa key, with the default name of id_rsa. If it does not exist
a new key will be created in this location.

rsa_ password is the password used to encrypt your rsa key. If your key is not encrypted,
then leave None.

Changes made here will not be ignored by git so do not push any senstive information.

"""

import getpass

user = {
    'rover_ip': '',
    'rover_user': 'ROVER USER NAME HERE',
    'rover_image_dir': '~/Pictures',  # using tilda '~' works here
    'basestation_dir': '/home/' + getpass.getuser() + '/Payload/',  # but not here
    'payload_file_subdir':'',  #names the dir of files the will be placed in payload
    'key_path': '/home/' + getpass.getuser() + '/.ssh/id_rsa',  # nor here
    'rsa_password': 'None'
}
