Installation and Usage
======================

Installation
-------------

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. `Ubuntu downloads can be found here <https://www.ubuntu.com/download/desktop>`_ . 
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
   * 2 CPU
   * 2 GB system memory
   * 25 GB of free hard drive space
  
  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
   * `ROS Kinetic <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_ if you have Ubuntu 16.04.
   * `ROS Indigo <http://wiki.ros.org/indigo/Installation/Ubuntu>`_ if you have Ubuntu 14.04.
* `Dataspeed DBW <https://bitbucket.org/DataspeedInc/dbw_mkz_ros>`_
   * Use this option to install the SDK on a workstation that already has ROS installed: `One Line SDK Install (binary) <https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default>`_
* Download the `Udacity Simulator <https://github.com/udacity/self-driving-car-sim/releases/tag/v0.1>`_.

Usage
-------

1. Clone the project repository

.. code-block:: bash

    git clone https://github.com/udacity/carla-driver.git


2. Install python dependencies

.. code-block:: bash

    cd carla-driver
    pip install -r requirements.txt


3. Install ROS dependencies

.. code-block:: bash

  sudo apt install ros-kinetic-pcl-ros


4. Make and run styx

.. code-block:: bash

  cd ros
  catkin_make
  source devel/setup.sh
  roslaunch launch/styx.launch


5. Run the simulator

.. code-block:: bash

  unzip linux_sys_int.zip (if you use linux)
  cd linux_sys_int
  chmod +x system_integration.x86_64
  ./system_integration.x86_64
