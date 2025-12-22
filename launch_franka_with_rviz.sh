#!/bin/bash

# 保存原始环境变量
ORIGINAL_LD_LIBRARY_PATH=$LD_LIBRARY_PATH
ORIGINAL_QT_PLUGIN_PATH=$QT_PLUGIN_PATH
ORIGINAL_QML2_IMPORT_PATH=$QML2_IMPORT_PATH

# 从LD_LIBRARY_PATH中移除CoppeliaSim路径，避免Qt冲突
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|:/home/william/CoppeliaSim||g' | sed 's|/home/william/CoppeliaSim:||g' | sed 's|/home/william/CoppeliaSim||g')

# 清理Qt相关的环境变量，防止加载CoppeliaSim的Qt插件
unset QT_PLUGIN_PATH
unset QML2_IMPORT_PATH
unset QT_QPA_PLATFORM_PLUGIN_PATH

# 设置使用系统的Qt插件路径
export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins

# 确保ROS环境已设置
# source /opt/ros/noetic/setup.bash
# source /home/william/jjma/catkin_ws/devel/setup.bash

echo "Modified LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "QT_PLUGIN_PATH: $QT_PLUGIN_PATH"
echo "Starting Franka controller with RViz..."

# 启动带RViz的controller
# roslaunch franka_example_controllers cartesian_impedance_example_controller.launch load_gripper:=true robot_ip:=172.16.0.2
roslaunch franka_example_controllers dual_impedance_controller.launch load_gripper:=true robot_ip:=172.16.0.2  arm_id:="panda"

# 恢复原始环境变量
export LD_LIBRARY_PATH=$ORIGINAL_LD_LIBRARY_PATH
export QT_PLUGIN_PATH=$ORIGINAL_QT_PLUGIN_PATH
export QML2_IMPORT_PATH=$ORIGINAL_QML2_IMPORT_PATH

