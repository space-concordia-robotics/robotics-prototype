import subprocess
from subprocess import Popen, PIPE
import os
import datetime
import flask
from flask import jsonify, make_response
import threading

import robot.basestation.ros_utils as ros_utils
from robot.basestation.ros_utils import fetch_ros_master_ip

global proc_video
proc_video = {}
active_recordings = []

def get_stream_shortname(stream_url):
    """ 
    Given the format http://localhost:8080/stream?topic=/FEED_NAME/image_raw

    Returns FEED_NAME
    """

    return stream_url.split("/")[-2]

def start_recording_feed(stream_url):
    """
    Start recording a feed given by a stream URL.

    Returns True if the stream starts successfully, false otherwise
    """

    stream_is_connected = is_stream_connected(stream_url)

    if stream_is_connected:
        active_recordings.append(stream_url)
        threading.Thread(target = start_ffmpeg_record, args = stream_url).start()
        return True, "Successfully started stream of " + get_stream_shortname(stream_url)
    else:
        return False, "Failed to connect to stream" + get_stream_shortname(stream_url)

def stop_recording_feed(stream_url):
    """
    Stop recording feed and handle potential errors
    """

    active_recordings.remove(stream_url)
    message = "Successfully stopped recording of " + get_stream_shortname(stream_url)
    success = True
    try:
        proc_video[stream_url].communicate(b'q')
    except (KeyError, ValueError):
        success = False
        message = "Failed to stop recording of " + get_stream_shortname(stream_url)

    return success, message

def is_recording_stream(stream_url):
    return stream_url in active_recordings

def is_stream_connected(stream_url):
    """
    Check if video feed is up before starting start ffmpeg

    Returns True on success, false otherwise
    """
    connection_check = os.system('ffprobe -select_streams v -i ' + stream_url)
    return connection_check == 0

def start_ffmpeg_record(stream_url):
    """
    Start ffmpeg to start recording stream
    """
    formatted_date = datetime.datetime.now().strftime("%Y_%m_%d_%I_%M_%S")
    filename = get_stream_filename(stream_url) + '_' + formatted_date
    save_video_dir = 'rover_stream/' + stream
    subprocess.Popen(['mkdir rover_stream'], shell=True)
    subprocess.Popen(['mkdir ' + save_video_dir], shell=True)
    proc_video[stream_url] = subprocess.Popen(['ffmpeg -i ' + stream_url + ' -acodec copy -vcodec copy ' + save_video_dir + '/' + filename + '.mp4'], stdin=PIPE, shell=True)
