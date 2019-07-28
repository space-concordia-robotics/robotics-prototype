## start_stream.sh
This uses mjpgstreamer, more details are documented in the bash script itself.
To start the stream, assuming you already have a usb web camera connected, run `./start_stream.sh`. Use `Ctrl + C` to stop the stream.

Assuming the stream is being dispatched, it can be accessed in any browser by visiting: `server_ip:8090/?action=stream`,
where `server_ip` is replaced with the actual ip address  of the server doing the streaming.
If you have started the stream, opening any page on the GUI should show the live of feed of the camera, assuming your ROS environment variables are properly setup.

### Capturing the stream to mp4

```
sudo apt install ffmpeg
ffmpeg -i "http://127.0.0.1:8090/?action=stream" -c:v libx264 -preset veryslow -crf 18 output.mp4
```

- `-crf 18` --> 6000 kb / s bitrate, excellent quality
- `-crf 30` --> 500 kb / s bitrate, medium quality
