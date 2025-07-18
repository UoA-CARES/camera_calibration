import cv2
import rclpy
from rclpy.node import Node
import os

from .base_nodes import MonoCalibrationNode, StereoCalibrationNode
from .checker_calibrator import CheckerCalibrator
from .charuco_calibrator import CharucoCalibrator

# Monocular cameras will also have R = the identity and P[1:3,1:3] = K.

# https://answers.opencv.org/question/187734/derivation-for-perspective-transformation-matrix-q/


# def compute_Q(camera_info_left, camera_info_right):
# Extract intrinsic parameters from left camera
# K_left = np.array(camera_info_left["camera_matrix"]).reshape(3, 3)
# P_left = np.array(camera_info_left["projection_matrix"]).reshape(3, 4)
# P_right = np.array(camera_info_right["projection_matrix"]).reshape(3, 4)


def create_checker_calibrator(paramater_node: Node, display: bool) -> CheckerCalibrator:
    paramater_node.declare_parameter("board_row", 6)
    paramater_node.declare_parameter("board_col", 9)
    paramater_node.declare_parameter("checker_size", 0.036)
    paramater_node.declare_parameter("scale_factor", 0.5)

    board_col = (
        paramater_node.get_parameter("board_col").get_parameter_value().integer_value
    )

    board_row = (
        paramater_node.get_parameter("board_row").get_parameter_value().integer_value
    )

    board_size = (board_col, board_row)

    checker_size = (
        paramater_node.get_parameter("checker_size").get_parameter_value().double_value
    )

    scale_factor = (
        paramater_node.get_parameter("scale_factor").get_parameter_value().double_value
    )

    return CheckerCalibrator(
        board_size=board_size,
        checker_size=checker_size,
        scale_factor=scale_factor,
        display=display,
    )


def create_charuco_calibrator(paramater_node: Node, display: bool) -> CharucoCalibrator:
    paramater_node.declare_parameter("board_row", 6)
    paramater_node.declare_parameter("board_col", 9)
    paramater_node.declare_parameter("checker_size", 0.036)
    paramater_node.declare_parameter("marker_size", 0.036)
    paramater_node.declare_parameter("dictionary_id", 0)
    paramater_node.declare_parameter("scale_factor", 0.5)

    board_col = (
        paramater_node.get_parameter("board_col").get_parameter_value().integer_value
    )

    board_row = (
        paramater_node.get_parameter("board_row").get_parameter_value().integer_value
    )

    board_size = (board_col, board_row)

    checker_size = (
        paramater_node.get_parameter("checker_size").get_parameter_value().double_value
    )

    marker_size = (
        paramater_node.get_parameter("marker_size").get_parameter_value().double_value
    )

    dictionary_id = (
        paramater_node.get_parameter("dictionary_id")
        .get_parameter_value()
        .integer_value
    )

    scale_factor = (
        paramater_node.get_parameter("scale_factor").get_parameter_value().double_value
    )

    paramater_node.get_logger().info("Calibrator Parameters:")
    paramater_node.get_logger().info(f"Board Row: {board_row}")
    paramater_node.get_logger().info(f"Board Col: {board_col}")
    paramater_node.get_logger().info(f"Board Size: {board_size}")
    paramater_node.get_logger().info(f"Checker Size: {checker_size}")
    paramater_node.get_logger().info(f"Marker Size: {marker_size}")
    paramater_node.get_logger().info(f"Dictionary ID: {dictionary_id}")

    return CharucoCalibrator(
        board_size=board_size,
        checker_size=checker_size,
        marker_size=marker_size,
        dictionary_id=dictionary_id,
        scale_factor=scale_factor,
        display=display,
    )


def create_stereo_node(
    temp_node: Node,
    calibrator: CheckerCalibrator,
    save_directory: str,
    diff_threshold: float,
) -> StereoCalibrationNode:

    temp_node.declare_parameter("left_camera_name", "left")
    temp_node.declare_parameter("right_camera_name", "right")

    left_camera_name = (
        temp_node.get_parameter("left_camera_name").get_parameter_value().string_value
    )
    right_camera_name = (
        temp_node.get_parameter("right_camera_name").get_parameter_value().string_value
    )

    return StereoCalibrationNode(
        calibrator=calibrator,
        left_camera_name=left_camera_name,
        right_camera_name=right_camera_name,
        save_directory=save_directory,
        diff_threshold=diff_threshold,
    )


def create_mono_node(
    temp_node: Node,
    calibrator: CheckerCalibrator,
    save_directory: str,
    diff_threshold: float,
) -> MonoCalibrationNode:
    temp_node.declare_parameter("camera_name", "left")

    camera_name = (
        temp_node.get_parameter("camera_name").get_parameter_value().string_value
    )

    return MonoCalibrationNode(
        calibrator=calibrator,
        camera_name=camera_name,
        save_directory=save_directory,
        diff_threshold=diff_threshold,
    )


def load_stereo_image_pairs(
    folder_path: str,
    prefix_left: str = "left_",
    prefix_right: str = "right_",
    ext: str = ".png",
):
    image_pairs = []

    # List all files
    files = os.listdir(folder_path)

    # Filter and sort to ensure matching order
    left_files = sorted(
        [f for f in files if f.startswith(prefix_left) and f.endswith(ext)]
    )
    right_files = sorted(
        [f for f in files if f.startswith(prefix_right) and f.endswith(ext)]
    )

    # Match by index (assumes naming like left_0.png, right_0.png)
    for left, right in zip(left_files, right_files):
        left_path = os.path.join(folder_path, left)
        right_path = os.path.join(folder_path, right)

        left_img = cv2.imread(left_path, cv2.IMREAD_COLOR)
        right_img = cv2.imread(right_path, cv2.IMREAD_COLOR)

        if left_img is not None and right_img is not None:
            image_pairs.append((left_img, right_img))

    return image_pairs


def load_mono_images(folder_path: str, ext: str = ".png"):
    images = []

    # List all files
    files = os.listdir(folder_path)

    # Filter and sort to ensure matching order
    image_files = sorted([f for f in files if f.endswith(ext)])

    # Load images
    for img_file in image_files:
        img_path = os.path.join(folder_path, img_file)
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)

        if img is not None:
            images.append(img)

    return images


def load_images(
    calibration_node: MonoCalibrationNode | StereoCalibrationNode, load_path: str
):
    calibration_node.get_logger().info(f"Loading images from {load_path}")

    if not os.path.exists(load_path):
        raise FileNotFoundError(f"Path does not exist: {load_path}")

    if isinstance(calibration_node, StereoCalibrationNode):
        stereo_images = load_stereo_image_pairs(load_path)
        calibration_node.get_logger().info(
            f"Loaded {len(stereo_images)} stereo image pairs"
        )

        for left_img, right_img in stereo_images:
            calibration_node.process_images(left_img, right_img)

    elif isinstance(calibration_node, MonoCalibrationNode):
        mono_images = load_mono_images(load_path)
        calibration_node.get_logger().info(f"Loaded {len(mono_images)} mono images")

        for img in mono_images:
            calibration_node.process_image(img)

    calibration_node.calibrate()


def live_capture(
    calibration_type: str,
    board_type: str,
    calibration_node: MonoCalibrationNode | StereoCalibrationNode,
):
    calibration_node.get_logger().info(
        f"{calibration_type} calibration is running with {board_type} board - push esc to exit or c to calibrate"
    )

    while rclpy.ok():
        # Process one callback if available
        rclpy.spin_once(calibration_node)

        key = cv2.waitKey(1)
        if key == ord("c"):
            calibration_node.calibrate()
            break
        elif key == 27:  # esc
            break


def main():
    rclpy.init()

    param_node = rclpy.create_node("parameter_loader")

    # Declare a parameter with a default value
    param_node.declare_parameter("calibration_type", "")
    param_node.declare_parameter("board_type", "")
    param_node.declare_parameter("diff_threshold", 0.0)
    param_node.declare_parameter(
        "save_directory", os.path.expanduser("~/calibration_images/")
    )
    param_node.declare_parameter("load_path", "")
    param_node.declare_parameter("display", True)

    calibration_type = (
        param_node.get_parameter("calibration_type").get_parameter_value().string_value
    )

    board_type = (
        param_node.get_parameter("board_type").get_parameter_value().string_value
    )

    save_directory = (
        param_node.get_parameter("save_directory").get_parameter_value().string_value
    )

    diff_threshold = (
        param_node.get_parameter("diff_threshold").get_parameter_value().double_value
    )

    load_path = param_node.get_parameter("load_path").get_parameter_value().string_value
    load_path = os.path.expanduser(load_path)

    display = param_node.get_parameter("display").get_parameter_value().bool_value

    calibrator: CheckerCalibrator | CharucoCalibrator
    if board_type == "checker":
        calibrator = create_checker_calibrator(param_node, display)
    elif board_type == "charuco":
        calibrator = create_charuco_calibrator(param_node, display)
    else:
        raise ValueError(f"Invalid board type: {board_type}")

    calibration_node: StereoCalibrationNode | MonoCalibrationNode
    if calibration_type == "stereo":
        calibration_node = create_stereo_node(
            param_node, calibrator, save_directory, diff_threshold
        )
    elif calibration_type == "mono":
        calibration_node = create_mono_node(
            param_node, calibrator, save_directory, diff_threshold
        )
    else:
        raise ValueError(f"Invalid calibration type: {calibration_type}")

    param_node.destroy_node()

    if load_path != "":
        load_images(calibration_node, load_path)
    else:
        live_capture(calibration_type, board_type, calibration_node)

    calibration_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
