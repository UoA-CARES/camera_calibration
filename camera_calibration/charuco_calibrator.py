import cv2
import numpy as np


class CharucoCalibrator:
    def __init__(
        self,
        board_size: tuple[int, int],
        checker_size: float,
        marker_size: float,
        dictionary_id: int,
        scale_factor: float,
        display: bool = True,
    ):
        self.board_size = board_size
        self.checker_size = checker_size
        self.marker_size = marker_size

        self.dictionary = cv2.aruco.Dictionary_get(dictionary_id)

        # Note charucoboard is the number of squares (corners +1) in each direction
        # For consistency with checkerboard with use the number of corners
        self.charuco_board = cv2.aruco.CharucoBoard_create(
            self.board_size[0] + 1,
            self.board_size[1] + 1,
            self.checker_size,
            self.marker_size,
            self.dictionary,
        )

        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # TODO incorporate this into the class
        self.scale_factor = scale_factor

        self.display = display

    def _balance_object_points(
        self,
        left_corner_ids: np.ndarray,
        left_corners: np.ndarray,
        right_corner_ids: np.ndarray,
        right_corners: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        balanced_ids = []

        balanced_left_corners = []
        balanced_right_corners = []

        for left_id, left_corner in zip(left_corner_ids, left_corners):
            for right_id, right_corner in zip(right_corner_ids, right_corners):
                if left_id == right_id:
                    balanced_ids.append(left_id)

                    balanced_left_corners.append(left_corner)
                    balanced_right_corners.append(right_corner)
                    continue

        return (
            np.array(balanced_ids),
            np.array(balanced_left_corners),
            np.array(balanced_right_corners),
        )

    def _detect_corners(
        self, image_bgr: np.ndarray, name: str
    ) -> tuple[bool, np.ndarray, np.ndarray]:

        marker_corners, marker_ids, rejected_corners = cv2.aruco.detectMarkers(
            image_bgr, self.dictionary, parameters=self.aruco_params
        )

        cv2.aruco.refineDetectedMarkers(
            image_bgr, self.charuco_board, marker_corners, marker_ids, rejected_corners
        )

        ret = False
        charuco_ids = np.array([])
        charuco_corners = np.array([])

        if len(marker_corners) > 0:
            # Refine marker detection to find Charuco corners
            ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, image_bgr, self.charuco_board
            )

            cv2.aruco.drawDetectedMarkers(image_bgr, marker_corners, marker_ids)
            cv2.aruco.drawDetectedCornersCharuco(
                image_bgr, charuco_corners, charuco_ids
            )

        if self.display:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 500, 500)
            cv2.imshow(name, image_bgr)

        return ret, charuco_ids, charuco_corners

    def _get_object_points(self, detected_charuco_ids: np.ndarray) -> np.ndarray:
        object_points = self.charuco_board.chessboardCorners

        return object_points[detected_charuco_ids]

    def process_image(
        self, image_bgr: np.ndarray, name: str
    ) -> tuple[bool, np.ndarray, np.ndarray]:

        ret, charuco_ids, charuco_corners = self._detect_corners(image_bgr, name)

        object_points = self._get_object_points(charuco_ids)

        return ret, charuco_corners, object_points

    def process_images(self, image_left_bgr: np.ndarray, image_right_bgr: np.ndarray):
        ret_left, left_ids, corners_left = self._detect_corners(image_left_bgr, "left")
        ret_right, right_ids, corners_right = self._detect_corners(
            image_right_bgr, "right"
        )

        balanced_corners_left = np.array([])
        balanced_corners_right = np.array([])
        object_points = np.array([])
        if ret_left and ret_right:
            (
                balanced_ids,
                balanced_corners_left,
                balanced_corners_right,
            ) = self._balance_object_points(
                left_ids, corners_left, right_ids, corners_right
            )

            if len(balanced_ids) > 0:
                object_points = self._get_object_points(balanced_ids)

        return (
            ret_left and ret_right and len(balanced_ids) > 5,
            balanced_corners_left,
            balanced_corners_right,
            object_points,
        )
