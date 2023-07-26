# Camera Utils

This package contains a C++ nodelet to publish a camera info loaded from a camera calibration file. Originally, this package was created to publish the camera info from the [EuRoC MAV dataset](https://paperswithcode.com/dataset/euroc-mav).

## Usage

Just make sure the camera calibration file is on the `config\camera_calibration` folder. An example to launch the nodelet:
```bash
export CAMERA_NAME="cam0"
roslaunch camera_utils info_publisher.launch camera_calibration_file:="euroc_mav_cam0.yaml"
```