import math
import os
from datetime import datetime

import cv2
import numpy as np
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from sensor_msgs.msg import Image

from .charuco_calibrator import CharucoCalibrator
from .checker_calibrator import CheckerCalibrator


def compute_Q(P1, P2):
    fx = P1[0, 0]
    cx = P1[0, 2]
    cy = P1[1, 2]

    # Extract baseline from right camera's projection matrix
    B = -P2[0, 3] / fx
    cx_prime = P2[0, 2]

    # Construct Q matrix
    Q = np.array(
        [
            [1, 0, 0, -cx],
            [0, 1, 0, -cy],
            [0, 0, 0, fx],
            [0, 0, 1 / B, (cx - cx_prime) / B],
        ]
    )
    return Q


def corners_similarity_score(
    corners: np.ndarray, corners_all: list[np.ndarray]
) -> float:
    min_diff = float("inf")

    for corners_other in corners_all:
        if corners.shape != corners_other.shape:
            continue

        total_diff = 0.0
        for corner, corner_other in zip(corners, corners_other):
            x_diff = corner[0][0] - corner_other[0][0]
            y_diff = corner[0][1] - corner_other[0][1]
            total_diff += math.sqrt(x_diff**2 + y_diff**2)

        if total_diff < min_diff:
            min_diff = total_diff

    min_diff = min_diff / corners.shape[0]

    return min_diff


def save_image(
    save_directory: str,
    name: str,
    image_bgr: np.ndarray,
) -> None:
    cv2.imwrite(f"{save_directory}/{name}.png", image_bgr)


class MonoCalibrationNode(Node):
    def __init__(
        self,
        calibrator: CheckerCalibrator | CharucoCalibrator,
        camera_name: str,
        save_directory: str,
        diff_threshold: float = 0,
    ):
        super().__init__("mono_calibration_node")

        self.calibrator = calibrator

        self.images: list[np.ndarry] = []

        date = datetime.now().strftime("%y_%m_%d_%H-%M-%S")

        self.save_directory = f"{save_directory}/{date}"
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        self.diff_threshold = diff_threshold

        self.image_size = (0, 0)

        self.corners = []
        self.object_points = []

        self.camera_name = camera_name

        self.create_subscription(
            Image, f"{self.camera_name}/image_color", self.image_callback, 10
        )

        self.cv_bridge = CvBridge()

    def process_images(self, image_bgr: np.ndarray):
        ret, corners, object_points = self.calibrator.process_image(image_bgr, "Image")

        if ret:
            sim_score = corners_similarity_score(corners, self.corners)

            if sim_score >= self.diff_threshold:
                self.image_size = image_bgr.shape[:2]

                self.corners.append(corners)
                self.object_points.append(object_points)

                save_image(self.save_directory, f"{len(self.corners)}", image_bgr)

                self.get_logger().info(f"Adding corners: {len(self.corners)}")

    def image_callback(self, image_msg: Image) -> None:
        image_bgr = self.cv_bridge.imgmsg_to_cv2(image_msg)

        self.process_images(image_bgr)

    def calibrate(self) -> None:
        self.get_logger().info(
            f"Calibrating with {len(self.corners)} depending on the number of images this can take milliseconds to minutes"
        )

        rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
            self.object_points, self.corners, self.image_size, None, None
        )

        # Monocular cameras have R = the identity and P[1:3,1:3] = K.
        R = np.eye(3)

        P = np.zeros((3, 4))
        P[0:3, 0:3] = camera_matrix

        self.get_logger().info(f"RMS: {rms}")

        # Write the calibration data to a YAML file
        output_file = self.save_directory + f"/{self.camera_name}_calibration.yaml"
        fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)

        fs.write("image_size", self.image_size)
        fs.write("rms", rms)
        fs.write("K", camera_matrix)
        fs.write("D", distortion)
        fs.write("R", R)
        fs.write("P", P)
        fs.release()

        self.get_logger().info("Calibration complete")


class StereoCalibrationNode(Node):

    def __init__(
        self,
        calibrator: CheckerCalibrator,
        left_camera_name: str,
        right_camera_name: str,
        save_directory: str,
        diff_threshold: float = 0,
    ):
        super().__init__("stereo_calibration_node")

        self.get_logger().info(f"Opencv version: {cv2.__version__}")

        self.calibrator = calibrator

        date = datetime.now().strftime("%y_%m_%d_%H-%M-%S")

        self.save_directory = f"{save_directory}/{date}"
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        self.diff_threshold = diff_threshold

        self.image_size = (0, 0)

        self.image_pairs: list[tuple[np.ndarry, np.ndarry]] = []
        self.left_corners = []
        self.right_corners = []
        self.object_points = []

        self.cv_bridge = CvBridge()

        self.left_camera_name = left_camera_name
        self.right_camera_name = right_camera_name

        self.left_image_sub = Subscriber(
            self, Image, f"{self.left_camera_name}/image_color"
        )
        self.right_image_sub = Subscriber(
            self, Image, f"{self.right_camera_name}/image_color"
        )

        self.sync = TimeSynchronizer(
            [self.left_image_sub, self.right_image_sub], queue_size=1
        )
        self.sync.registerCallback(self.SyncCallback)

    def process_images(self, image_left_bgr: np.ndarray, image_right_bgr: np.ndarray):
        ret, corners_left, corners_right, object_points = (
            self.calibrator.process_images(
                image_left_bgr.copy(), image_right_bgr.copy()
            )
        )

        if ret:
            left_sim_score = corners_similarity_score(corners_left, self.left_corners)
            right_sim_score = corners_similarity_score(
                corners_right, self.right_corners
            )

            if (
                left_sim_score >= self.diff_threshold
                and right_sim_score >= self.diff_threshold
            ):
                self.image_size = image_left_bgr.shape[:2]
                self.left_corners.append(corners_left)
                self.right_corners.append(corners_right)
                self.object_points.append(object_points)

                save_image(
                    self.save_directory,
                    f"left_{len(self.left_corners)}",
                    image_left_bgr,
                )
                save_image(
                    self.save_directory,
                    f"right_{len(self.right_corners)}",
                    image_right_bgr,
                )

                self.get_logger().info(
                    f"Adding corners: {len(self.left_corners)} {len(self.right_corners)}"
                )

    def SyncCallback(self, image_left_msg: Image, image_right_msg: Image) -> None:
        image_left_bgr = self.cv_bridge.imgmsg_to_cv2(image_left_msg)
        image_right_bgr = self.cv_bridge.imgmsg_to_cv2(image_right_msg)

        self.process_images(image_left_bgr, image_right_bgr)

    def _save_calibration(
        self,
        camera_name: str,
        rms: float,
        stereo_rms: float,
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
        R: np.ndarray,
        P: np.ndarray,
    ) -> None:
        output_file = self.save_directory + f"/{camera_name}_calibration.yaml"
        fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)

        fs.write("image_size", self.image_size)
        fs.write("rms", rms)
        fs.write("stereo_rms", stereo_rms)

        fs.write("K", camera_matrix)
        fs.write("D", distortion)
        fs.write("R", R)
        fs.write("P", P)
        fs.release()

    def calibrate(self):

        self.get_logger().info(
            f"Calibrating with {len(self.left_corners)} depending on the number of images this can take milliseconds to minutes"
        )

        left_rms, left_camera_matrix, left_dist, _, _ = cv2.calibrateCamera(
            self.object_points,
            self.left_corners,
            self.calibrator.board_size,
            None,
            None,
        )
        right_rms, right_camera_matrix, right_dist, _, _ = cv2.calibrateCamera(
            self.object_points,
            self.right_corners,
            self.calibrator.board_size,
            None,
            None,
        )

        self.get_logger().info(f"Left RMS: {left_rms} Right RMS: {right_rms}")

        (
            stereo_rms,
            camera_matrix_left,
            distortion_left,
            camera_matrix_right,
            distortion_right,
            R,
            T,
            _,
            _,
        ) = cv2.stereoCalibrate(
            self.object_points,
            self.left_corners,
            self.right_corners,
            left_camera_matrix,
            left_dist,
            right_camera_matrix,
            right_dist,
            self.image_size,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
            flags=cv2.CALIB_FIX_INTRINSIC,
        )

        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
            camera_matrix_left,
            distortion_left,
            camera_matrix_right,
            distortion_right,
            self.image_size,
            R,
            T,
            alpha=0,
            flags=cv2.CALIB_ZERO_DISPARITY,
        )

        self.get_logger().info(f"Stereo RMS: {stereo_rms}")

        # Write the calibration data to a YAML file
        self._save_calibration(
            self.left_camera_name,
            left_rms,
            stereo_rms,
            camera_matrix_left,
            distortion_left,
            R1,
            P1,
        )

        self._save_calibration(
            self.right_camera_name,
            right_rms,
            stereo_rms,
            camera_matrix_right,
            distortion_right,
            R2,
            P2,
        )

        self.get_logger().info("Calibration complete")
