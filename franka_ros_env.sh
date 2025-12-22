#!/bin/bash

echo "设置Franka ROS环境..."

# 禁用Anaconda环境（如果存在）
if command -v conda >/dev/null 2>&1; then
    echo "禁用Anaconda环境..."
    conda deactivate 2>/dev/null || true
    # 移除conda路径
    export PATH=$(echo $PATH | tr ':' '\n' | grep -v conda | grep -v anaconda | tr '\n' ':' | sed 's/:$//')
fi

# 设置代理（如果需要）
export http_proxy=http://172.16.0.10:7890
export https_proxy=http://172.16.0.10:7890
export HTTP_PROXY=http://172.16.0.10:7890
export HTTPS_PROXY=http://172.16.0.10:7890

# 设置ROS环境
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    echo "ROS Noetic环境已加载"
fi

# 设置catkin工作空间（如果存在且已编译）
if [ -f ~/jjma/catkin_ws/devel/setup.bash ]; then
    source ~/jjma/catkin_ws/devel/setup.bash
    echo "Franka catkin工作空间已加载"
fi

# 确保libfranka路径可用
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH"

echo "Franka ROS环境设置完成！"
echo "当前Python: $(which python3)"
echo "当前ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
