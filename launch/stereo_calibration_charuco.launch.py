from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace"
    )

    declare_left_camera_name = DeclareLaunchArgument(
        "left_camera_name", default_value="left", description="Name of the left camera"
    )

    declare_right_camera_name = DeclareLaunchArgument(
        "right_camera_name",
        default_value="right",
        description="Name of the right camera",
    )

    declare_board_row = DeclareLaunchArgument(
        "board_row", default_value="10", description="Corner Rows of the checkerboard"
    )

    declare_board_col = DeclareLaunchArgument(
        "board_col", default_value="7", description="Corner Coloms of the checkerboard"
    )

    declare_checker_size = DeclareLaunchArgument(
        "checker_size",
        default_value="100.0",
        description="Size of the checkerboard square (mm)",
    )

    declare_marker_size = DeclareLaunchArgument(
        "marker_size", default_value="78.0", description="Size of the markers (mm)"
    )

    declare_dictionary_id = DeclareLaunchArgument(
        "dictionary_id", default_value="3", description="Aruco Dictionary ID"
    )

    declare_diff_threshold = DeclareLaunchArgument(
        "diff_threshold",
        default_value="50.0",
        description="Difference threshold for similar board poses (pixels)",
    )

    declare_scale_factor = DeclareLaunchArgument(
        "scale_factor",
        default_value="0.5",
        description="Scale factor to improve detection time",
    )

    declare_save_directory = DeclareLaunchArgument(
        "save_path",
        default_value="",
        description="Path to save camera calibration files - will save as camera_name_calibration.yaml files",
    )

    declare_display = DeclareLaunchArgument(
        "display", default_value="true", description="Display images"
    )

    namespace = LaunchConfiguration(declare_namespace.name)
    left_camera_name = LaunchConfiguration(declare_left_camera_name.name)
    right_camera_name = LaunchConfiguration(declare_right_camera_name.name)
    board_row = LaunchConfiguration(declare_board_row.name)
    board_col = LaunchConfiguration(declare_board_col.name)
    checker_size = LaunchConfiguration(declare_checker_size.name)
    marker_size = LaunchConfiguration(declare_marker_size.name)
    dictionary_id = LaunchConfiguration(declare_dictionary_id.name)
    diff_threshold = LaunchConfiguration(declare_diff_threshold.name)
    scale_factor = LaunchConfiguration(declare_scale_factor.name)
    save_directory = LaunchConfiguration(declare_save_directory.name)
    display = LaunchConfiguration(declare_display.name)

    return LaunchDescription(
        [
            declare_namespace,
            declare_left_camera_name,
            declare_right_camera_name,
            declare_board_row,
            declare_board_col,
            declare_checker_size,
            declare_marker_size,
            declare_dictionary_id,
            declare_diff_threshold,
            declare_scale_factor,
            declare_save_directory,
            declare_display,
            Node(
                package="camera_calibration",
                executable="calibration_node",
                name="calibration_node",
                output="screen",
                namespace=namespace,
                parameters=[
                    {
                        declare_left_camera_name.name: left_camera_name,
                        declare_right_camera_name.name: right_camera_name,
                        declare_board_row.name: board_row,
                        declare_board_col.name: board_col,
                        declare_checker_size.name: checker_size,
                        declare_marker_size.name: marker_size,
                        declare_dictionary_id.name: dictionary_id,
                        declare_diff_threshold.name: diff_threshold,
                        declare_scale_factor.name: scale_factor,
                        declare_save_directory.name: save_directory,
                        declare_display.name: display,
                    },
                    {"board_type": "charuco"},
                    {"calibration_type": "stereo"},
                ],
            ),
        ]
    )
