#!/bin/bash

# Fix protobuf/gazebo compatibility issue for Franka ROS
# This script temporarily disables franka_gazebo to allow building other packages

set -e

echo "=== Fixing Protobuf/Gazebo Compatibility Issue ==="

cd /home/william/franka/catkin_ws

# Source the environment
sh /home/william/jjma/catkin_ws/franka_ros_env.sh

echo "Creating CATKIN_IGNORE for franka_gazebo to skip compilation..."
touch src/franka_ros/franka_gazebo/CATKIN_IGNORE

echo "Cleaning build files..."
catkin_make clean

echo "Building without franka_gazebo..."
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/usr/local/lib/cmake/Franka

echo "Checking if build was successful..."
if [ $? -eq 0 ]; then
    echo "✅ Build successful! franka_ros packages compiled without Gazebo simulation."
    echo "✅ Real robot controllers should work fine."
    echo ""
    echo "📝 Note: Gazebo simulation is disabled due to protobuf compatibility issues."
    echo "   To re-enable simulation, remove: src/franka_ros/franka_gazebo/CATKIN_IGNORE"
    echo "   and fix the protobuf version compatibility first."
else
    echo "❌ Build failed even without franka_gazebo. Check for other issues."
    exit 1
fi

echo ""
echo "=== Next Steps ==="
echo "1. Source the environment: source ~/franka_ros_env.sh"
echo "2. Test the controller: ./test_franka_complete_final.sh"
echo "3. Or launch manually: roslaunch franka_example_controllers cartesian_impedance_example_controller_no_gui_final.launch robot_ip:=172.16.0.2"
