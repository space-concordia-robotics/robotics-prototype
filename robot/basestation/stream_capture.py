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

def start_recording_feed(stream):
    """
    Setup variables and start connection check
    """
    recording_log_msg = 'recording feed of ' + stream
    formatted_date = datetime.datetime.now().strftime("%Y_%m_%d_%I_%M_%S")
    return feed_connection_check(stream, formatted_date, recording_log_msg)

def stop_recording_feed(stream):
    """
    Stop recording feed and handle potential errors
    """
    recording_log_msg = 'saved recording of ' + stream
    error_state = 0
    try:
        proc_video[stream].communicate(b'q')
    except (KeyError, ValueError):
        error_state = 1

    return jsonify(recording_log_msg=recording_log_msg, error_state=error_state)

def feed_connection_check(stream, formatted_date, recording_log_msg):
    """
    Check if video feed is up before starting start ffmpeg
    """
    stream_url = "http://" + fetch_ros_master_ip() + ":8080/stream?topic=/cv_camera/image_raw"
    error_state = 0
    connection_check = os.system('ffprobe -select_streams v -i ' + stream_url)
    if connection_check == 0:
        threading.Thread(target = start_ffmpeg_record, args = (stream, stream_url, formatted_date)).start()
        print(recording_log_msg)
        return jsonify(recording_log_msg=recording_log_msg, error_state=error_state)

    else:
        recording_log_msg = 'failed to connect to video feed of ' + stream
        error_state = 1
        print(recording_log_msg)
        return jsonify(recording_log_msg=recording_log_msg, error_state=error_state)

def start_ffmpeg_record(stream, stream_url, formatted_date):
    """
    Start ffmpeg to start recording stream

    The stream_url varible in this function will have to be modified in order
    to handle recording multiple streams simultaneously
    """
    filename = stream + '_' + formatted_date
    save_video_dir = 'rover_stream/' + stream
    subprocess.Popen(['mkdir rover_stream'], shell=True)
    subprocess.Popen(['mkdir ' + save_video_dir], shell=True)
    proc_video[stream] = subprocess.Popen(['ffmpeg -i ' + stream_url + ' -acodec copy -vcodec copy ' + save_video_dir + '/' + filename + '.mp4'], stdin=PIPE, shell=True)
