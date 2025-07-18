# camera_calibration
ROS package for calibrating cameras via Checker or Charuco boards - stereo or mono camera systems.

## Installation Instructions
You will first need to install the Pylon software (>= 8.0) from [here](https://www.baslerweb.com/en/software/pylon/) 

`git clone`the repository into your desired ros2 workspace directory on your local machine.

Run `pip3 install -r requirements' in the **root directory** of this package.

`colcon build` your ros2 workspace. 

# Usage
This package can be utilised to calibrate stereo or mono (single) camera setups.
Camera calibration can be done live with image capture or by processing a precapture set of images. 
Calibration can be done using a Checker board or Charuco board - make sure to have all board configurations entered correctly.
The recommneded utilisation of this package is via the launch files.

## Run Parameters
The node can be configured with the parameters below.

**namespace**: Namespace to run node under

**scale_factor**: Resize images for calibration for faster processing - defaults to full size (1.0)

**diff_threshold**: Minimum difference between points in images for diverse range of images

**save_path**: Path to save calibration data and images - defaults to "~/calibration_images"

**load_path**: Path to load prior calibration data if not capturing live

**board_type**: Checker or Charuco

**calibration_type**: Mono or Stereo

### Checker Board Parameters

**board_row**: Number of corners on the Checker board rows

**board_col**: Number of corners on the Checker board cols

**checker_size**: Size of the checker board squares

### Charuco Board Parameters

**board_row**: Number of corners on the Checker board rows (note this is corners to be consistent with checker boards)

**board_col**: Number of corners on the Checker board cols (note this is corners to be consistent with checker boards)

**checker_size**: Size of the checker board squares

**marker_size**: Size of the Arcuo markers on the board

**dictionary_id**: Dictionary ID of the Aruco markers on the board.

### Mono Camera Calibration

**camera_name**: Name of the camera being calibrated

```sh
ros2 launch camera_calibration mono_calibration_charuco.launch.py 
```

```sh
ros2 launch camera_calibration mono_calibration_checker.launch.py
```

### Stereo Camera Camera Calibration

**left_camera_name**: name of the left camera in the stereo pair

**right_camera_name**: name of the right camera in the stereo pair

```sh
ros2 launch camera_calibration stereo_calibration_charuco.launch.py 
```

```sh
ros2 launch camera_calibration stereo_calibration_checker.launch.py
```
