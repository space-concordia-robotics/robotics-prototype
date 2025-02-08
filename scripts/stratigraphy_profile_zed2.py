import cv2
import numpy as np
import pyzed.sl as sl


def capture_zed2_image_and_depth():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.MILLIMETER

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        exit(1)

    image = sl.Mat()
    depth_map = sl.Mat()

    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)

    image_np = image.get_data()[:, :, :3]  # Drop alpha channel
    depth_np = depth_map.get_data()

    zed.close()

    return image_np, depth_np


def extract_stratigraphic_profile(depth_map, column_x):
    """Extracts depth values along a vertical line (column_x)."""
    height = depth_map.shape[0]
    profile = [depth_map[y, column_x] for y in range(height)]
    return np.array(profile)


def overlay_profile_on_image(image, profile, column_x):
    """Overlay the stratigraphic profile onto the image."""
    overlay = image.copy()
    height, _, _ = image.shape
    max_depth = np.nanmax(profile)  # Ignore NaN values

    # Normalize profile depth values for overlay
    profile_normalized = (profile / max_depth) * height
    profile_normalized = height - profile_normalized.astype(int)  # Flip for display

    # Draw the profile as a red line
    for y in range(height - 1):
        if not np.isnan(profile[y]) and not np.isnan(profile[y + 1]):
            cv2.line(overlay, (column_x, profile_normalized[y]),
                     (column_x, profile_normalized[y + 1]), (0, 0, 255), 2)

    return overlay


def main():
    image, depth_map = capture_zed2_image_and_depth()

    # Choose a column in the middle of the image for the profile
    column_x = image.shape[1] // 2
    profile = extract_stratigraphic_profile(depth_map, column_x)

    # Overlay profile onto the image
    overlay_image = overlay_profile_on_image(image, profile, column_x)

    cv2.imshow("Stratigraphic Profile", overlay_image)
    cv2.imwrite("stratigraphic_profile.png", overlay_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
