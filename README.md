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

## Citation
```
@article{gao2023lfvb,
  title={LFVB-BioSLAM: A bionic SLAM system with a light-weight LiDAR front end and a bio-inspired visual back end},
  author={Gao, Ruilan and Wan, Zeyu and Guo, Sitong and Jiang, Changjian and Zhang, Yu},
  journal={Biomimetics},
  volume={8},
  number={5},
  pages={410},
  year={2023},
  publisher={MDPI}
}
```