import cv2
import numpy as np


class CheckerCalibrator:
    def __init__(
        self,
        board_size: tuple[int, int],
        checker_size: float,
        scale_factor: float,
        display: bool = True,
    ):
        self.board_size = board_size
        self.checker_size = checker_size

        self.scale_factor = scale_factor

        self.display = display

    def _detect_corners(
        self, image_bgr: np.ndarray, name: str
    ) -> tuple[bool, np.ndarray]:
        iamge_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

        scaled_image_grey = cv2.resize(
            iamge_gray, (0, 0), fx=self.scale_factor, fy=self.scale_factor
        )

        flags = cv2.CALIB_CB_FAST_CHECK
        ret, corners = cv2.findChessboardCorners(
            scaled_image_grey, self.board_size, flags
        )

        if ret:
            corners = corners * (1 / self.scale_factor)

            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

            corners = cv2.cornerSubPix(
                iamge_gray, corners, (11, 11), (-1, -1), criteria
            )

            # Draw and display the corners
            cv2.drawChessboardCorners(image_bgr, self.board_size, corners, ret)

        if self.display:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 500, 500)
            cv2.imshow(name, image_bgr)

        return ret, corners

    def _get_object_points(self) -> np.ndarray:
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        object_points = np.zeros(
            (self.board_size[0] * self.board_size[1], 3), np.float32
        )
        object_points[:, :2] = np.mgrid[
            0 : self.board_size[0], 0 : self.board_size[1]
        ].T.reshape(-1, 2)
        object_points = object_points * self.checker_size
        return object_points

    def process_image(
        self, image_bgr: np.ndarray, name: str
    ) -> tuple[bool, np.ndarray, np.ndarray]:

        ret, corners = self._detect_corners(image_bgr, name)

        object_points = self._get_object_points()

        return ret, corners, object_points

    def process_images(self, image_left_bgr: np.ndarray, image_right_bgr: np.ndarray):
        ret_left, corners_left = self._detect_corners(image_left_bgr, "left")
        ret_right, corners_right = self._detect_corners(image_right_bgr, "right")

        object_points = self._get_object_points()

        return ret_left and ret_right, corners_left, corners_right, object_points
