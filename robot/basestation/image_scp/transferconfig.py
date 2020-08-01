#! usr/bin/env python3
"""
Update user credentials here for username to access the rover, the rover ip address,
diectory where the images are located on the rover, and the destination directory on the base
station.

Basestation_dir will make a new directory with the name entered here and copy the
contents from the rover_image_dir.

key_path is the path to your rsa key, with the default name of id_rsa. If it does not exist
a new key will be created in this location.

rsa_ password is the password used to encrypt your rsa key. If your key is not encrypted,
then leave None.

Changes made here will not be ignored by git so do not push any senstive information.



"""

import getpass
import datetime

user = {'rover_ip':'ip',
        'username': 'username',
        'rover_image_dir': 'rover_dir',
        'basestation_dir':'basestation_dir'+str(datetime.datetime.now()
                                                .strftime("_%Y-%m-%d_%H:%M:%S")),
        'key_path': '/home/'+ getpass.getuser()+'/.ssh/id_rsa',
        'rsa_password':'None'
        }
