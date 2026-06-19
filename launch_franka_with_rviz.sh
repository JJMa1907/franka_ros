#!/usr/bin/env bash
# If invoked via sh, re-exec with bash to keep consistent behavior.
if [ -z "${BASH_VERSION:-}" ]; then
  exec bash "$0" "$@"
fi

# 保存原始环境变量
ORIGINAL_LD_LIBRARY_PATH=$LD_LIBRARY_PATH
ORIGINAL_QT_PLUGIN_PATH=$QT_PLUGIN_PATH
ORIGINAL_QML2_IMPORT_PATH=$QML2_IMPORT_PATH

# 从LD_LIBRARY_PATH中移除CoppeliaSim路径，避免Qt冲突
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | sed 's|:/home/william/CoppeliaSim||g' | sed 's|/home/william/CoppeliaSim:||g' | sed 's|/home/william/CoppeliaSim||g')
export LD_LIBRARY_PATH="/home/william/franka/libfranka_0.15/build:/home/william/jjma/catkin_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu"

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

# 如果当前没有ROS Master，则自动启动 roscore
if ! timeout 2s rostopic list >/dev/null 2>&1; then
  echo "ROS master not detected, starting roscore..."
  roscore >/tmp/roscore_franky.log 2>&1 &
  ROSCORE_PID=$!

  for _ in $(seq 1 30); do
    if timeout 2s rostopic list >/dev/null 2>&1; then
      echo "ROS master is up (pid: ${ROSCORE_PID})"
      break
    fi
    sleep 0.5
  done

  if ! timeout 2s rostopic list >/dev/null 2>&1; then
    echo "[ERROR] Failed to start ROS master. Check /tmp/roscore_franky.log" >&2
    exit 1
  fi
fi

# 可选后端:
#   BACKEND=ros_control (默认): 使用原 ros_control 控制器
#   BACKEND=franky_compat: 使用 franky + ROS 兼容话题桥接
BACKEND=${BACKEND:-ros_control}

if [ "$BACKEND" = "ros_control" ]; then
  echo "Starting ROS control cartesian impedance controller..."
  # roslaunch franka_example_controllers cartesian_impedance_example_controller.launch load_gripper:=true robot_ip:=172.16.0.2 "$@"
  roslaunch franka_example_controllers dual_impedance_controller.launch load_gripper:=true robot_ip:=172.16.0.2  arm_id:="panda"
else
  echo "Starting franky ROS compatibility bridge..."
  roslaunch franka_example_controllers cartesian_impedance_example_controller.launch backend:=franky load_gripper:=true robot_ip:=172.16.0.2 "$@"
fi

# 恢复原始环境变量
export LD_LIBRARY_PATH=$ORIGINAL_LD_LIBRARY_PATH
export QT_PLUGIN_PATH=$ORIGINAL_QT_PLUGIN_PATH
export QML2_IMPORT_PATH=$ORIGINAL_QML2_IMPORT_PATH
