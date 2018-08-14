# bin/bash
#rosrun camera_calibration cameracalibrator.py  --size 8x6 --square 0.0392 --pattern 'chessboard'  image:=/gmsl_camera/port_0/cam_1/image_raw camera:=/gmsl_camera/port_0/cam_1
rosrun camera_calibration_fisheye cameracalibrator.py  --size 8x6 --square 0.0392  image:=/gmsl_camera/port_0/cam_2/image_raw camera:=/gmsl_camera/port_0/cam_2

