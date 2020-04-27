import subprocess
from subprocess import Popen, PIPE
import os
import datetime
import flask
from flask import jsonify, make_response
import threading
from robot.util.utils import run_shell
import robot.basestation.ros_utils as ros_utils
from robot.basestation.ros_utils import fetch_ros_master_ip
import ffmpeg

global proc_video
proc_video = {}
active_recordings = []
ROVER_STREAM_FOLDER = "rover_stream"
IMAGES_FOLDER = "stream_images"

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

    global active_recordings
    global video_filename
    stream_shortname = get_stream_shortname(stream_url)

    save_video_dir = ROVER_STREAM_FOLDER + "/" + get_stream_shortname(stream_url)
    if not os.path.exists(save_video_dir):
      os.makedirs(save_video_dir)

    stream_is_connected = is_stream_connected(stream_url)

    formatted_date = datetime.datetime.now().strftime("%Y_%m_%d_%I_%M_%S")
    video_filename = get_stream_shortname(stream_url) + '_' + formatted_date + ".mp4"
    video_filename = save_video_dir + '/' + video_filename

    if stream_is_connected:
        print("Started recording " + stream_shortname)
        active_recordings.append(stream_shortname)
        threading.Thread(target = start_ffmpeg_record, args = (stream_url, video_filename)).start()
        return True, "Successfully started recording " + stream_shortname + " stream at " + os.path.abspath(video_filename)
    else:
        return False, "Failed to connect to " + stream_shortname + " stream"

def stop_recording_feed(stream_url, camera_rotation):
    """
    Stop recording feed and handle potential errors
    """

    global active_recordings
    stream_shortname = get_stream_shortname(stream_url)
    active_recordings.remove(stream_shortname)
    message = "Successfully stopped recording of " + stream_shortname
    success = True
    try:
        proc_video[stream_url].communicate(b'q')
        if camera_rotation != '0':
            rotate_stream(video_filename, camera_rotation)

    except (KeyError, ValueError) as e:
        print(e)
        success = False
        message = "Failed to stop recording of " + stream_shortname

    print(message)
    return success, message

def rotate_stream(filename, rotation):
    """
    Rotate stream to the rotation of the GUI
    """
    if 'jpg' in filename:
        temp_filename = filename + '.jpg'
    elif 'mp4' in filename:
        temp_filename = filename + '.mp4'

    stream = ffmpeg.input(filename)
    # If rotation is 180 degrees, use vflip. Else use transpose filter
    if rotation == '1':
        stream = ffmpeg.filter_(stream, 'transpose', 1)
    elif rotation == '2':
        stream = ffmpeg.filter_(stream, 'vflip')
    elif rotation == '3':
        stream = ffmpeg.filter_(stream, 'transpose', 2)

    stream = ffmpeg.output(stream, temp_filename)
    ffmpeg.run(stream)
    os.remove(filename)
    os.rename(temp_filename, filename)

def is_recording_stream(stream_url):
    global active_recordings
    return get_stream_shortname(stream_url) in active_recordings

def is_stream_connected(stream_url):
    """
    Check if video feed is up before starting start ffmpeg

    Returns True on success, false otherwise
    """
    connection_check = os.system('ffprobe -select_streams v -i ' + stream_url)
    return connection_check == 0

def start_ffmpeg_record(stream_url, filename):
    """
    Start ffmpeg to start recording stream
    """
    proc_video[stream_url] = subprocess.Popen(['ffmpeg -i ' + stream_url + ' -acodec copy -vcodec copy ' + filename], stdin=PIPE, shell=True)

def stream_capture(stream_url, camera_rotation):
    """ Given a stream, captures an image.

        stream_url : The URL of the stream to capture the image.
    """
    image_directory = IMAGES_FOLDER + "/" + get_stream_shortname(stream_url) + "/"

    print("Capturing image of " + stream_url);

    if not os.path.exists(image_directory):
      os.makedirs(image_directory)

    formatted_date = datetime.datetime.now().strftime("%Y_%m_%d_%I_%M_%S");
    image_filename =  get_stream_shortname(stream_url) + "_" + formatted_date + ".jpg"
    image_filename = image_directory + image_filename

    error, output = run_shell("ffmpeg -i " + stream_url + " -ss 00:00:01.500 -f image2 -vframes 1 " + image_filename)

    # Rotate image
    if camera_rotation != '0':
        rotate_stream(image_filename, camera_rotation)

    message = "Successfully captured image " + os.path.abspath(image_filename)

    if error:
        message = "Failed to capture image of " + stream_url
    print(message);

    return not error, message
