
Carla Driver
============

- This is the capstone project for Udacity's Self Driving Car Nanodegree. This
  code is aimed to run in an actual physical self-drive vehicle.
 
[![Build Status](https://travis-ci.org/kung-fu-panda-automotive/carla-driver.svg?branch=master)](https://travis-ci.org/kung-fu-panda-automotive/carla-driver)

### Team Members

This repository is maintained by the following:
- [Lukasz Janyst](https://github.com/ljanyst) (xyz@jany.st)
- [Mithi Sevilla](https://github.com/mithi) (mithi.sevilla@gmail.com)
- [Maurice Loskyll](https://github.com/mauricelos) (maurice@loskyll.de)
- [Kostas Oreopoulos](https://github.com/buffos) (kostas.oreopoulos@gmail.com)

### Installation 

* Install ROS Kinetic
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: 
    [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/carla-driver.git
```

2. Install python dependencies
```bash
cd carla-driver
pip install -r requirements.txt
```

3. Install ROS dependencies
```bash
sudo apt install ros-kinetic-pcl-ros
```

4. Download the TensorFlow models to your `~/.ros` directory

  * [Classifier](https://s3-eu-west-1.amazonaws.com/ljanyst-udacity/traffic-lights-classifier.pb)
  * [Detector](https://s3-eu-west-1.amazonaws.com/ljanyst-udacity/traffic-lights-detector-faster-r-cnn.pb)

5. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

6. Run the simulator
```bash
unzip linux_sys_int.zip (if you use linux)
cd linux_sys_int
chmod +x system_integration.x86_64
./system_integration.x86_64
```
