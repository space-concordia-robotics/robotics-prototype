#!/bin/bash

# To get access to the camera you typically need to be part of the "video" group.
# The problem is that the "video" group has a different id different distros.
# Meaning that if you simply add the user inside the container to the "video" group and the id is different
# than that of the "video" group on the host, the user won't be added to the "video" group that actually has access to the camera (the one on the host)

# If you're on Ubuntu you won't have that problem since the container is based off an ubuntu image (video group id is 44)

videoid=$1
if [ $videoid -ne 44 ]
then
    echo "Delete video group and recreate it with same id as host"
    groupdel video
    groupadd -g $videoid video
fi
