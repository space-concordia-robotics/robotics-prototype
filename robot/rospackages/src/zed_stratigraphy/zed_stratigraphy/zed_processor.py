import numpy as np
import pyzed.sl as sl


class ZEDProcessor:
    def __init__(self):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.camera_resolution = sl.RESOLUTION.HD1080

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Failed to open ZED2 camera")

    def capture_depth(self):
        depth = sl.Mat()

        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            depth_data = depth.get_data()
            return depth_data
        return None

    def extract_stratigraphy(self, depth_data):
        depth_data = np.nan_to_num(depth_data, nan=0.0)
        avg_depth_per_row = np.mean(depth_data, axis=1)
        return avg_depth_per_row.tolist()
