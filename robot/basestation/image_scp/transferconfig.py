#! usr/bin/env python3
"""
Update user credentials here for username to access the rover, the rover ip address,
diectory where the images are located on the rover, and the destination directory on the base
station. basestation_dir will make a new directory with the name entered here and copy the
contents from the rover image directory

Note that a rsa key pair must be set up between the rover and basestation for this script to
work

Details listed here will not be ignored by git

"""
user = {'rover_ip':'',
        'username': '',
        'rover_image_dir': '',
        'basestation_dir':''
        }
