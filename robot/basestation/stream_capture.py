import subprocess
from subprocess import Popen, PIPE
import os
import datetime
import flask
from flask import jsonify, make_response
import threading

import ros_utils
from ros_utils import fetch_ros_master_ip

def start_recording_feed(stream):
    recording_log_msg = 'recording feed of ' + stream
    formatted_date = datetime.datetime.now().strftime("%I:%M:%S_%B_%d_%Y")
    filename = stream + '_' + formatted_date
    save_video_dir = 'rover_stream/' + stream
    stream_url = "http://" + fetch_ros_master_ip() + ":8080/stream?topic=/cv_camera/image_raw"
    return feed_connection_check(stream, filename, save_video_dir, stream_url, recording_log_msg)

def stop_recording_feed(stream):
    recording_log_msg = 'saved recording of ' + stream
    proc_video[stream].communicate(b'q')
    return jsonify(recording_log_msg=recording_log_msg)

def feed_connection_check(stream, filename, save_video_dir, stream_url, recording_log_msg):
    error_state = 0
    connection_check = os.system('ffprobe -show_streams -select_streams v -i ' + stream_url)
    if connection_check == 0:
        threading.Thread(target = start_ffmpeg_record, args = (stream, stream_url, filename, save_video_dir)).start()
        print(recording_log_msg)
        return jsonify(recording_log_msg=recording_log_msg, error_state=error_state)

    else:
        recording_log_msg = 'failed to connect to video feed of ' + stream
        error_state = 1
        print(recording_log_msg)
        return jsonify(recording_log_msg=recording_log_msg, error_state=error_state)

def start_ffmpeg_record(stream, stream_url, filename, save_video_dir):
    """
    the stream_url varible in this function will have to be modified in order
    to be able to record multiple streams simultaneously
    """
    subprocess.Popen(['mkdir rover_stream'], shell=True)
    subprocess.Popen(['mkdir ' + save_video_dir], shell=True)
    global proc_video
    proc_video = {}
    proc_video[stream] = subprocess.Popen(['ffmpeg -i ' + stream_url + ' -acodec copy -vcodec copy ' + save_video_dir + '/' + filename + '.mp4'], stdin=PIPE, shell=True)
