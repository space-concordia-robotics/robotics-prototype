"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    poll_delay_seconds - how many seconds to wait between captures
    camera_index - which camera index to open
    camera_destination_index - If present, will attempt to use v4l2loopback 
                                to allow another process to access the camera.
                                Needs ffmpeg and v4l2loopback-dev installed.
    camera_matrix - Array corresponding to the 3x3 camera's intrinsic matrix K
    distortion_coefficients - Array of distortion coefficients

Author: Nathan Sprague, Marc Scattolin
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import subprocess

def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
    corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, -marker_size / 2, 0],
                            [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


class ArucoNode(rclpy.node.Node):
    def declare_params(self):
        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="poll_delay_seconds",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="How long to wait between captures",
            ),
        )

        self.declare_parameter(
            name="camera_index",
            value=0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Which camera index to open.",
            ),
        )

        self.declare_parameter(
            name="camera_destination_index",
            value=-1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Which index under /dev/video to send camera stream to for other devices.",
            ),
        )

        self.declare_parameter(
            name="camera_matrix",
            value=[0.],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="Camera intrinsic matrix flattened to an array",
            ),
        )

        self.declare_parameter(
            name="distortion_coefficients",
            value=[0.],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="Camera distortion coefficients",
            ),
        )
    
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.video_capture.release()
        if self.ffmpeg_process:
            self.ffmpeg_process.terminate()
    
    def setup_camera_loopback(self):
        self.get_logger().info(f"Camera destination index: {self.camera_destination_index}")
        completion1 = subprocess.run(["sudo", "modprobe", "-r", "v4l2loopback"], capture_output=True)
        completion2 = subprocess.run(["sudo", "modprobe", "v4l2loopback", f"video_nr={self.camera_destination_index}", 
                                        "card_label=Video-Loopback", "exclusive_caps=1"], capture_output=True)
        self.ffmpeg_process = subprocess.Popen(["ffmpeg", "-i", f"/dev/video{self.camera_index}", "-f", "v4l2", "-codec:v", "rawvideo", "-pix_fmt", "yuv420p", f"/dev/video{self.camera_destination_index}"])
        self.camera_index = self.camera_destination_index

        if completion1.returncode != 0:
            self.get_logger().error(f"Error setting up multicamera: {completion1.stderr}")
        if completion2.returncode != 0:
            self.get_logger().error(f"Error setting up multicamera: {completion2.stderr}")


    def __init__(self):
        super().__init__("aruco_node")
        self.ffmpeg_process = self.video_capture = None

        # Declare params in separate function to de-clutter __init__
        self.declare_params()

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        poll_delay_seconds = (
            self.get_parameter("poll_delay_seconds").get_parameter_value().double_value
        )
        self.get_logger().info(f"Poll frequency: {poll_delay_seconds}")

        self.camera_index = (
            self.get_parameter("camera_index").get_parameter_value().integer_value
        )
        self.get_logger().info(f"Camera index: {self.camera_index}")

        self.camera_destination_index = (
            self.get_parameter("camera_destination_index").get_parameter_value().integer_value
        )
        if self.camera_destination_index != -1:
            self.setup_camera_loopback()

        self.camera_matrix = (
            self.get_parameter("camera_matrix").get_parameter_value().double_array_value
        )
        if len(self.camera_matrix) == 9:
            self.camera_matrix = np.reshape(np.array(self.camera_matrix), (3,3))
        else:
            self.camera_matrix = None

        self.distortion_coefficients = (
            self.get_parameter("distortion_coefficients").get_parameter_value().double_array_value
        )
        if len(self.distortion_coefficients) >= 4: 
            self.distortion_coefficients = np.array(self.distortion_coefficients)
        else:
            self.distortion_coefficients = None

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError as e:
            self.get_logger().error(
                "bad aruco_dictionary_id: '{}'".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        # Setup timers for opening camera (to allow easy retry) and for detecting aruco tags
        self.detect_timer = self.create_timer(poll_delay_seconds, self.image_callback)
        self.open_video_timer = self.create_timer(1.0, self.cam_callback)

        if cv2.__version__ < "4.7.0":
            self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        else:
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)

    def cam_callback(self):
        try:
            self.video_capture = cv2.VideoCapture(self.camera_index)
            # Once successful, don't try to open again
            self.open_video_timer.cancel()
        except:
            self.get_logger().warn(f"Could not open camera at {self.camera_index}, trying again")
            return

    def image_callback(self):
        if self.video_capture is None:
            self.get_logger().info(f"Still waiting on video stream")
            return
        if not self.video_capture.isOpened():
            self.get_logger().warn("Video stream not opened for aruco detection")
            return

        rval, frame = self.video_capture.read()
        
        cv_image = frame
        markers = ArucoMarkers()
        pose_array = PoseArray()

        markers.header.stamp = self.get_clock().now().to_msg()

        if cv2.__version__ < "4.7.0":
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
            )
        else:
            corners, marker_ids, rejected = self.detector.detectMarkers(cv_image)

        if marker_ids is not None:
            if self.camera_matrix is not None and self.distortion_coefficients is not None:
                rvecs, tvecs, _ = estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.distortion_coefficients
                )

                for i, marker_id in enumerate(marker_ids):
                    pose = Pose()
                    pose.position.x = tvecs[i][0][0]
                    pose.position.y = tvecs[i][1][0]
                    pose.position.z = tvecs[i][2][0]

                    # rot_matrix = np.eye(4)
                    # rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs))[0]
                    # quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                    # pose.orientation.x = quat[0]
                    # pose.orientation.y = quat[1]
                    # pose.orientation.z = quat[2]
                    # pose.orientation.w = quat[3]

                    pose_array.poses.append(pose)
                    markers.poses.append(pose)
                    markers.marker_ids.append(marker_id[0])
            else:
                for marker_id in marker_ids:
                    markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)


def main():
    rclpy.init()

    # This node uses other resources (opencv camera, process for ffmpeg)
    # So cleanup (outside of destroy_node) is needed.
    with ArucoNode() as node:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
