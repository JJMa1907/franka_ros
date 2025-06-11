# ROS integration for Franka Robotics research robots

[![CI](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml)


See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
## install 
After setting up ROS Noetic, create a Catkin workspace in a directory of your choice:

cd /path/to/desired/folder
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/noetic/setup.sh
catkin_init_workspace src
Then clone the franka_ros repository from GitHub:

git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
By default, this will check out the newest release of franka_ros. If you want to build a particular version of franka_ros instead, check out the corresponding Git tag:

git checkout <version>
Install any missing dependencies and build the packages:
``` shell
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.sh
```
Note: If you have installed a franka ros before, you can get libfranka in its build cmakecache.txt
Warning

If you also installed ros-noetic-libfranka, libfranka might be picked up from /opt/ros/noetic instead of from your custom libfranka build!




## use
```
source devel/setup.sh
```

rostopic pub test

``` shell
rostopic pub /joint_impedance_example_controller/joint_command std_msgs/Float64MultiArray "data: [1., -0.16, -0.23, -1.96, -0.152, 1.8, 1.98, 0.04, 0.04]"
```