#  ros_gmsl_driver
A ROS driver for GMSL cameras on [Drive PX platform](https://www.nvidia.com/en-us/self-driving-cars/drive-platform/)

For now, better performance than master.

Flow is: Frame Capture on NVmedia->CUDA->CPU->Publish on ROS
## Performance
* Raw images 1920x1208px: tested with 4 cameras giving ~20FPS on same Tegra as driver

