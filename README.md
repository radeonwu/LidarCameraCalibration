# LidarCameraCalibration

An existing code using ROS and PCL library \
https://github.com/heethesh/lidar_camera_calibration

An another ROS pkg, \
http://wiki.ros.org/but_calibration_camera_velodyne \
https://github.com/robofit/but_velodyne

An Matlab reference code \
https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox

# test with docker #
## install dependancy ##
apt-get install ros-kinetic-ros-numpy \
apt-get install ros-kinetic-tf2-sensor-msgs \
apt-get install python-tk \
apt-get install python-pip \
pip install --upgrade pip \
python -m pip install -U matplotlib \
this will install version 2.x

##run calib ##
rosrun auro_calibration calibrate_camera_lidar.py --calibrate
