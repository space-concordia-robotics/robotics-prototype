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

def start_recording_feed(stream_url):
    """
    Setup variables and start connection check
    """
    recording_log_msg = 'recording feed of ' + stream_url
    return feed_connection_check(stream, formatted_date, recording_log_msg)

def stop_recording_feed(stream_url):
    """
    Stop recording feed and handle potential errors
    """

    active_recordings.remove(stream_url)
    recording_log_msg = 'saved recording of ' + stream
    error_state = 0
    try:
        proc_video[stream_url].communicate(b'q')
    except (KeyError, ValueError):
        error_state = 1

    return jsonify(recording_log_msg=recording_log_msg, error_state=error_state)

def is_stream_recording(stream_url):
    return stream_url in active_recordings

def feed_connection_check(stream_url, recording_log_msg):
    """
    Check if video feed is up before starting start ffmpeg
    """
    error_state = 0
    connection_check = os.system('ffprobe -select_streams v -i ' + stream_url)
    if connection_check == 0:
        threading.Thread(target = start_ffmpeg_record, args = ( stream_url, formatted_date)).start()
        print(recording_log_msg)
        return jsonify(recording_log_msg=recording_log_msg, error_state=error_state)

    else:
        recording_log_msg = 'failed to connect to video feed of ' + stream_url
        error_state = 1
        print(recording_log_msg)
        return jsonify(recording_log_msg=recording_log_msg, error_state=error_state)

def start_ffmpeg_record(stream_url):
    """
    Start ffmpeg to start recording stream
    """
    formatted_date = datetime.datetime.now().strftime("%Y_%m_%d_%I_%M_%S")
    filename = 'stream_' + formatted_date
    save_video_dir = 'rover_stream/' + stream
    subprocess.Popen(['mkdir rover_stream'], shell=True)
    subprocess.Popen(['mkdir ' + save_video_dir], shell=True)
    active_recordings.append(stream_url)
    proc_video[stream] = subprocess.Popen(['ffmpeg -i ' + stream_url + ' -acodec copy -vcodec copy ' + save_video_dir + '/' + filename + '.mp4'], stdin=PIPE, shell=True)
