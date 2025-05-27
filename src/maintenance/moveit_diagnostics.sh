#!/bin/bash
# Diagnostic script to check MoveIt-Controller connection

echo "=== MoveIt Controller Connection Diagnostic ==="
echo "This script will help identify why MoveIt can't find your controllers"
echo

# Step 1: Check if controllers are actually running
echo "Step 1: Checking running controllers..."
echo "Running 'ros2 control list_controllers':"
ros2 control list_controllers
echo

# Step 2: Check controller manager namespace
echo "Step 2: Checking controller manager namespace..."
echo "Active controller managers:"
ros2 node list | grep -i controller_manager || echo "No controller_manager found!"
echo

# Step 3: Check MoveIt's view of controllers
echo "Step 3: Checking MoveIt's controller knowledge..."
echo "This requires MoveIt to be running. Checking move_group node:"
ros2 node list | grep move_group || echo "move_group not running!"
echo

# Step 4: Check action servers
echo "Step 4: Checking FollowJointTrajectory action servers..."
echo "Available action servers:"
ros2 action list | grep follow_joint_trajectory || echo "No follow_joint_trajectory action found!"
echo

# Step 5: Check joint states topic
echo "Step 5: Checking joint states..."
echo "Joint states topic info:"
ros2 topic info /joint_states -v
echo

# Step 6: Check parameter server for controller config
echo "Step 6: Checking controller parameters..."
echo "Arm controller parameters:"
ros2 param list /arm_controller 2>/dev/null || echo "arm_controller parameters not accessible"
echo

# Step 7: Test trajectory execution capability
echo "Step 7: Testing trajectory interface..."
echo "Checking if arm_controller provides action interface:"
timeout 2 ros2 action info /arm_controller/follow_joint_trajectory 2>/dev/null || echo "Action interface not available"
echo

echo "=== Diagnostic Complete ==="
echo
echo "Common issues and solutions:"
echo "1. If controllers aren't listed in Step 1:"
echo "   - Controllers aren't actually running"
echo "   - Launch controller.launch.py first"
echo
echo "2. If action servers aren't found in Step 4:"
echo "   - Controller isn't configured for trajectory following"
echo "   - Check controller type in proto3_controllers.yaml"
echo
echo "3. If MoveIt can't see controllers:"
echo "   - Namespace mismatch between MoveIt and controllers"
echo "   - Controller names in moveit_controllers.yaml don't match actual names"