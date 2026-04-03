# LFVB-BioSLAM: A Bionic SLAM System with a Light-Weight LiDAR Front End and a Bio-Inspired Visual Back End

## Install
```
mkdir -p ws_slam/src
cd ws_slam/src
git clone https://github.com/rlgao/LFVB_BioSLAM.git
cd LFVB_BioSLAM
catkin_make
source devel/setup.bash
roslaunch ratslam_ros **.launch
```