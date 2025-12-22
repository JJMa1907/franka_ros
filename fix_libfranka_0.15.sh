#!/bin/bash

# Fix Franka libfranka version incompatibility 
# Server version 9 requires libfranka >= 0.15.0
# This script upgrades libfranka from 0.14.x to 0.15.x

set -e

echo "=== Franka libfranka 0.15.0 升级脚本 ==="
echo "解决 Server version 9 与 Library version 8 的兼容性问题"
echo

# Source environment first
sh /home/william/jjma/catkin_ws/franka_ros_env.sh


echo "1. 检查当前状态..."
echo "当前 libfranka 版本: $(pkg-config --modversion libfranka 2>/dev/null || echo '未知')"

# Check internet connectivity through proxy
echo "2. 检查网络连接..."
if curl -x 172.16.0.10:7890 -s --connect-timeout 10 https://github.com > /dev/null; then
    echo "✅ 网络连接正常 (通过代理)"
else
    echo "❌ 网络连接失败 - 请检查代理设置"
    exit 1
fi

# echo "3. 备份当前 libfranka..."
# if [ -d "/home/william/franka/libfranka" ]; then
#     cp -r /home/william/franka/libfranka /home/william/franka/libfranka_0.14_backup
#     echo "✅ 已备份到 libfranka_0.14_backup"
# fi

# echo "4. 下载 libfranka 0.15.x..."
# cd /home/william/franka

# # Remove existing libfranka_0.15 if exists
# if [ -d "libfranka_0.15" ]; then
#     rm -rf libfranka_0.15
# fi

# # Clone libfranka 0.15
# git clone --recursive https://github.com/frankaemika/libfranka.git libfranka_0.15
# cd libfranka_0.15

# echo "5. 切换到最新的 0.15.x 版本..."
# # Get the latest 0.15.x tag
# LATEST_015=$(git tag -l "0.15.*" | sort -V | tail -n1)
# if [ -z "$LATEST_015" ]; then
#     echo "使用 master 分支 (可能包含 0.15.x 版本)"
#     git checkout master
# else
#     echo "使用版本: $LATEST_015"
#     git checkout $LATEST_015
# fi

# # Update submodules
# git submodule update --init --recursive

# echo "5.1 修复 CMake 版本兼容性问题..."
# # Fix CMake version compatibility issue
# sed -i 's/cmake_minimum_required(VERSION 3.0.2)/cmake_minimum_required(VERSION 3.5)/' common/CMakeLists.txt
# echo "CMake 版本兼容性已修复"

# echo "6. 编译 libfranka 0.15.x..."
# mkdir -p build && cd build

# # Configure with CMake (adding policy version to fix CMake 4.x compatibility)
# cmake .. \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DBUILD_TESTS=OFF \
#     -DBUILD_EXAMPLES=ON \
#     -DCMAKE_INSTALL_PREFIX=/usr/local \
#     -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# # Build
# make -j$(nproc)

# echo "7. 安装 libfranka 0.15.x..."
# sudo make install

# # Update library cache
# sudo ldconfig

echo "8. 验证安装..."
NEW_VERSION=$(pkg-config --modversion libfranka 2>/dev/null || echo '检测失败')
echo "新安装的 libfranka 版本: $NEW_VERSION"

if pkg-config --exists libfranka; then
    echo "✅ libfranka 安装成功"
else
    echo "❌ libfranka 安装可能有问题"
    exit 1
fi

echo "9. 重新编译 franka_ros..."
cd /home/william/jjma/catkin_ws

# Clean previous build
catkin_make clean

# Rebuild with new libfranka
echo "正在重新编译 franka_ros 包..."
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/usr/local/lib/cmake/Franka

if [ $? -eq 0 ]; then
    echo "✅ franka_ros 重新编译成功"
else
    echo "❌ franka_ros 编译失败"
    echo "请检查编译错误并手动解决"
    exit 1
fi

# echo "10. 测试新版本..."
# cd /home/william/franka/libfranka_0.15/build

# echo "可用的测试程序:"
# echo "  - ./examples/communication_test <robot_ip>  # 测试通信"
# echo "  - ./examples/echo_robot_state <robot_ip>    # 获取机器人状态"

# echo
# echo "=== 升级完成 ==="
# echo "✅ libfranka 已升级到 $NEW_VERSION"
# echo "✅ franka_ros 已重新编译"
# echo "✅ 现在应该兼容 Server version 9"
# echo
# echo "下一步测试:"
# echo "1. 测试连接: cd /home/william/franka/libfranka_0.15/build && ./examples/communication_test 172.16.0.2"
# echo "2. 启动控制器: roslaunch franka_example_controllers cartesian_impedance_example_controller_no_gui_final.launch robot_ip:=172.16.0.2"
# echo
# echo "⚠️  注意: 每次使用前请运行 'source ~/franka_ros_env.sh'"
