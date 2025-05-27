#!/bin/bash
# Quick verification script

echo "=== ROS2 Humble Verification ==="

# Check environment
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"

# Check key packages
echo ""
echo "=== Checking Key Packages ==="
packages=("gazebo_ros2_control" "joint_trajectory_controller" "moveit" "robot_state_publisher")

for pkg in "${packages[@]}"; do
    if ros2 pkg list | grep -q "$pkg"; then
        echo "✅ $pkg: FOUND"
    else
        echo "❌ $pkg: NOT FOUND"
    fi
done

# Check Gazebo
echo ""
echo "=== Checking Gazebo ==="
if command -v gazebo &> /dev/null; then
    echo "✅ Gazebo: $(gazebo --version 2>/dev/null | head -1)"
else
    echo "❌ Gazebo: NOT FOUND"
fi

# Check plugin file
plugin_file="/opt/ros/humble/lib/libgazebo_ros2_control.so"
if [[ -f "$plugin_file" ]]; then
    echo "✅ gazebo_ros2_control plugin: FOUND"
else
    echo "❌ gazebo_ros2_control plugin: NOT FOUND"
fi

echo ""
echo "=== Test Complete ==="