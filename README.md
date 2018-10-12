# Particle Localization #
This package implements a particle filter 6DOF localization algorithm using RGB-D camera in known 3D environments. 
We use octomap and 3D Euclidean Distance Map for map . This method uses external visual odometry
 (such as fovis, demo) to predict the pose of the particle set, then update each particle's weight using 
Likelihood observation model.

This package depends on the message defined in [range-flow odometry](https://github.com/shichaoy/rangeflow_odom) package. The odometry provides initial guess then the localization corrects the drift.

<br>

**Related Paper:**

* **Robust Autonomous Flight in Constrained and Visually Degraded Shipboard Environments**, JFR 2017, ICRA 2015, Z. Fang, S. Yang, et al. S. Scherer  [**PDF**](http://www.frc.ri.cmu.edu/~syang/Publications/JFR_2016_ship.pdf)

If you use the code in your research work, please cite the above paper. Please do not hesitate to contact the authors if you have any further questions.

<br>


## Installation

1. Install octomap > 1.5
(This should not be necessary. Just need to install octomap packages. The dynamicedt3d augments the missing functionality in the indigo packages.)
```
git clone git://github.com/OctoMap/octomap.git

cd octomap; mkdir build; cd build;
cmake ..; make; sudo make install
```
2. Install octomap_ros

```sudo apt-get install ros-indigo-octomap-ros  ros-indigo-octomap-msgs```

3. ```catkin_make``` using ROS


<br>

## Usage

1. Subscribed Topics:

```point cloud``` or ```depth image.```

optional ```imu``` (only roll and pitch will be used, if parameter use_imu is enabled)



2. Published Topics:

```pose```: The final localization pose.

```~particlecloud```: The complete particle distribution


See ```launch``` files in launch folder for details on how to use it.

<br>

#### Author & Contacts
Zheng Fang(fangzheng@mail.neu.edu.cn), Shichao Yang(shichaoy@andrew.cmu.edu), Sebastian Scherer(basti@andrew.cmu.edu)
