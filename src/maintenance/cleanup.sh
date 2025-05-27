#!/bin/bash
# Deep clean script for ROS2 zombie processes
# Run with: bash deep_clean_ros2.sh

echo "=== ROS2 Deep Clean Script ==="
echo "This will forcefully clean all ROS2 processes and nodes"
echo

# Function to kill processes by name pattern
kill_by_pattern() {
    local pattern=$1
    local pids=$(pgrep -f "$pattern" 2>/dev/null)
    if [ ! -z "$pids" ]; then
        echo "Found processes matching '$pattern': $pids"
        for pid in $pids; do
            echo "  Killing PID $pid..."
            kill -9 $pid 2>/dev/null
        done
    fi
}

# Step 1: Show current problematic nodes
echo "Current duplicate nodes:"
ros2 node list 2>/dev/null | sort | uniq -d
echo

# Step 2: Kill all ROS2 control related processes
echo "Step 1: Killing ROS2 control processes..."
kill_by_pattern "ros2_control_node"
kill_by_pattern "controller_manager"
kill_by_pattern "spawner"
kill_by_pattern "joint_state_broadcaster"
kill_by_pattern "arm_controller"
kill_by_pattern "gz_ros2_control"

# Step 3: Kill Gazebo if running
echo -e "\nStep 2: Killing Gazebo processes..."
kill_by_pattern "gzserver"
kill_by_pattern "gzclient"
kill_by_pattern "gz sim"
kill_by_pattern "ign gazebo"

# Step 4: Kill any ROS2 launch processes
echo -e "\nStep 3: Killing ROS2 launch processes..."
kill_by_pattern "ros2 launch"
kill_by_pattern "_ros2_launch"

# Step 5: Stop and restart ROS2 daemon
echo -e "\nStep 4: Restarting ROS2 daemon..."
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 2

# Step 6: Clean temporary files and locks
echo -e "\nStep 5: Cleaning temporary files..."
rm -rf /tmp/ros2_*
rm -rf /tmp/.ros2_*
rm -rf ~/.ros/log/*

# Step 7: Clear shared memory segments (if any)
echo -e "\nStep 6: Clearing shared memory..."
# List and remove shared memory segments owned by current user
ipcs -m | grep $(whoami) | awk '{print $2}' | while read id; do
    if [ ! -z "$id" ] && [ "$id" != "shmid" ]; then
        ipcrm -m $id 2>/dev/null && echo "  Removed shared memory segment $id"
    fi
done

# Step 8: Final verification
echo -e "\nStep 7: Verification..."
echo "Remaining ROS2 processes:"
ps aux | grep -E "(ros2|gazebo|controller)" | grep -v grep || echo "  None found (good!)"

echo -e "\nCurrent ROS2 nodes:"
ros2 node list 2>/dev/null || echo "  No nodes running (good!)"

echo -e "\n=== Clean complete! ==="
echo "You should now be able to launch your system cleanly."
echo
echo "Tips for preventing this issue:"
echo "1. Always use Ctrl+C only once and wait for clean shutdown"
echo "2. Use 'ros2 launch --noninteractive' to prevent hanging on shutdown"
echo "3. Consider adding shutdown handlers to your launch files"