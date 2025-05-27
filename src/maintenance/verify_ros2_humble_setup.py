#!/usr/bin/env python3
"""
ROS2 Humble Environment Verification Script for Proto3 Robot Project
This script verifies all required plugins, packages, and configurations
for your 6-DOF robot arm synthetic data generation pipeline.

Usage: python3 verify_ros2_humble_setup.py
"""

import subprocess
import sys
import os
import json
from pathlib import Path
import xml.etree.ElementTree as ET

class Colors:
    """ANSI color codes for terminal output"""
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

class ROS2HumbleVerifier:
    def __init__(self):
        self.results = {
            'environment': {},
            'packages': {},
            'plugins': {},
            'controllers': {},
            'hardware_interfaces': {},
            'gazebo': {},
            'moveit': {},
            'sensors': {}
        }
        
    def run_command(self, cmd, capture=True, timeout=30):
        """Run a shell command and return output"""
        try:
            if capture:
                result = subprocess.run(
                    cmd, shell=True, capture_output=True, 
                    text=True, timeout=timeout
                )
                return result.returncode == 0, result.stdout, result.stderr
            else:
                result = subprocess.run(cmd, shell=True, timeout=timeout)
                return result.returncode == 0, "", ""
        except subprocess.TimeoutExpired:
            return False, "", "Command timeout"
        except Exception as e:
            return False, "", str(e)

    def print_header(self, title):
        """Print a formatted section header"""
        print(f"\n{Colors.BLUE}{'='*70}{Colors.RESET}")
        print(f"{Colors.BLUE}{Colors.BOLD}{title}{Colors.RESET}")
        print(f"{Colors.BLUE}{'='*70}{Colors.RESET}")

    def print_status(self, message, status, details=""):
        """Print a status message with color"""
        if status:
            icon = f"{Colors.GREEN}✓{Colors.RESET}"
        else:
            icon = f"{Colors.RED}✗{Colors.RESET}"
        
        print(f"{icon} {message}")
        if details:
            print(f"   {Colors.CYAN}→ {details}{Colors.RESET}")

    def check_environment(self):
        """Check ROS2 environment setup"""
        self.print_header("ROS2 Environment Verification")
        
        # Check ROS2 distribution
        ros_distro = os.environ.get('ROS_DISTRO', 'Not set')
        is_humble = ros_distro == 'humble'
        self.print_status(f"ROS2 Distribution: {ros_distro}", is_humble)
        self.results['environment']['ros_distro'] = ros_distro
        
        # Check ROS2 domain
        ros_domain = os.environ.get('ROS_DOMAIN_ID', 'Not set')
        self.print_status(f"ROS Domain ID: {ros_domain}", True)
        self.results['environment']['ros_domain'] = ros_domain
        
        # Check essential environment variables
        env_vars = [
            'ROS_LOCALHOST_ONLY',
            'AMENT_PREFIX_PATH',
            'PYTHONPATH',
            'LD_LIBRARY_PATH'
        ]
        
        for var in env_vars:
            value = os.environ.get(var, 'Not set')
            is_set = value != 'Not set'
            self.print_status(f"{var}", is_set, value[:100] + "..." if len(value) > 100 else value)
            self.results['environment'][var] = value

    def check_core_packages(self):
        """Check installation of core ROS2 packages"""
        self.print_header("Core ROS2 Packages Verification")
        
        # Essential packages for your robot project
        required_packages = [
            # Core ROS2
            'ros2cli',
            'rclcpp',
            'rclpy',
            'std_msgs',
            'geometry_msgs',
            'sensor_msgs',
            'tf2_ros',
            'tf2_geometry_msgs',
            
            # Robot description and visualization
            'robot_state_publisher',
            'joint_state_publisher',
            'joint_state_publisher_gui',
            'urdf',
            'xacro',
            'rviz2',
            
            # Gazebo (Classic for Humble)
            'gazebo_ros_pkgs',
            'gazebo_ros2_control',
            'gazebo_plugins',
            
            # ROS2 Control
            'ros2_control',
            'ros2_controllers',
            'controller_manager',
            'joint_state_broadcaster',
            'joint_trajectory_controller',
            'position_controllers',
            'effort_controllers',
            
            # MoveIt
            'moveit_ros_planning_interface',
            'moveit_ros_move_group',
            'moveit_kinematics',
            'moveit_planners_ompl',
            'moveit_servo',
            
            # Additional utilities
            'rosbag2',
            'plotjuggler_ros',
            'rqt_robot_steering',
            'rqt_joint_trajectory_controller'
        ]
        
        for package in required_packages:
            success, stdout, stderr = self.run_command(f"ros2 pkg list | grep -w {package}")
            self.print_status(f"Package: {package}", success)
            self.results['packages'][package] = success

    def check_gazebo_plugins(self):
        """Check Gazebo Classic plugins for Humble"""
        self.print_header("Gazebo Classic Plugins Verification")
        
        # Check Gazebo installation
        success, stdout, stderr = self.run_command("gazebo --version")
        if success:
            version = stdout.strip()
            self.print_status("Gazebo Classic installed", True, version)
            self.results['gazebo']['version'] = version
        else:
            self.print_status("Gazebo Classic installed", False, stderr)
            return
        
        # Check for gazebo_ros2_control plugin (correct for Humble)
        gazebo_plugins_path = "/opt/ros/humble/lib"
        plugin_file = f"{gazebo_plugins_path}/libgazebo_ros2_control.so"
        plugin_exists = os.path.exists(plugin_file)
        self.print_status("gazebo_ros2_control plugin", plugin_exists, plugin_file)
        self.results['gazebo']['ros2_control_plugin'] = plugin_exists
        
        # Check other essential Gazebo plugins
        gazebo_plugin_files = [
            "libgazebo_ros_camera.so",
            "libgazebo_ros_depth_camera.so",
            "libgazebo_ros_imu_sensor.so",
            "libgazebo_ros_joint_state_publisher.so",
            "libgazebo_ros_force_torque.so"
        ]
        
        for plugin in gazebo_plugin_files:
            plugin_path = f"{gazebo_plugins_path}/{plugin}"
            exists = os.path.exists(plugin_path)
            self.print_status(f"Plugin: {plugin}", exists, plugin_path if exists else "Not found")
            self.results['gazebo'][plugin] = exists

    def check_hardware_interfaces(self):
        """Check available hardware interfaces"""
        self.print_header("ROS2 Control Hardware Interfaces")
        
        # List available hardware interfaces
        success, stdout, stderr = self.run_command("ros2 control list_hardware_interfaces")
        if success:
            self.print_status("Hardware interface command", True)
            print(f"   {Colors.CYAN}Available interfaces:{Colors.RESET}")
            for line in stdout.split('\n')[:10]:  # Show first 10 lines
                if line.strip():
                    print(f"   {Colors.WHITE}• {line.strip()}{Colors.RESET}")
        else:
            self.print_status("Hardware interface command", False, stderr)
        
        self.results['hardware_interfaces']['command_available'] = success
        
        # Check for specific hardware interface types we need
        required_interfaces = [
            'gazebo_ros2_control/GazeboSystem',
            'mock_components/GenericSystem'
        ]
        
        # We'll check these when we can actually query the plugins
        success, stdout, stderr = self.run_command("ros2 pkg list | grep -E '(gazebo_ros2_control|mock_components)'")
        for interface in required_interfaces:
            pkg_name = interface.split('/')[0]
            pkg_exists = pkg_name in stdout if success else False
            self.print_status(f"Hardware Interface: {interface}", pkg_exists)
            self.results['hardware_interfaces'][interface] = pkg_exists

    def check_controllers(self):
        """Check available controllers"""
        self.print_header("ROS2 Controllers Verification")
        
        # Required controllers for your 6-DOF arm
        required_controllers = [
            'joint_state_broadcaster/JointStateBroadcaster',
            'joint_trajectory_controller/JointTrajectoryController',
            'position_controllers/JointPositionController',
            'effort_controllers/JointEffortController',
            'velocity_controllers/JointVelocityController'
        ]
        
        # Check controller packages
        for controller in required_controllers:
            pkg_name = controller.split('/')[0]
            success, stdout, stderr = self.run_command(f"ros2 pkg list | grep -w {pkg_name}")
            self.print_status(f"Controller: {controller}", success)
            self.results['controllers'][controller] = success

    def check_moveit_setup(self):
        """Check MoveIt installation and configuration"""
        self.print_header("MoveIt Planning Framework Verification")
        
        # Core MoveIt packages
        moveit_packages = [
            'moveit_core',
            'moveit_ros_planning',
            'moveit_ros_planning_interface',
            'moveit_ros_move_group',
            'moveit_planners_ompl',
            'moveit_kinematics',
            'moveit_servo'
        ]
        
        for package in moveit_packages:
            success, stdout, stderr = self.run_command(f"ros2 pkg list | grep -w {package}")
            self.print_status(f"MoveIt Package: {package}", success)
            self.results['moveit'][package] = success
        
        # Check for OMPL planners
        success, stdout, stderr = self.run_command("ros2 run moveit_planners_ompl moveit_generate_state_database --help")
        self.print_status("OMPL planners available", success)
        self.results['moveit']['ompl_available'] = success

    def check_sensor_plugins(self):
        """Check sensor plugins for synthetic data generation"""
        self.print_header("Sensor Plugins for Synthetic Data Generation")
        
        # Camera and sensor plugins essential for your use case
        sensor_plugins = [
            "libgazebo_ros_camera.so",           # RGB camera
            "libgazebo_ros_depth_camera.so",     # Depth camera  
            "libgazebo_ros_multicamera.so",      # Multi-camera setup
            "libgazebo_ros_openni_kinect.so",    # Kinect-style sensor
            "libgazebo_ros_laser.so",            # LiDAR
            "libgazebo_ros_gpu_laser.so",        # GPU-accelerated LiDAR
            "libgazebo_ros_imu_sensor.so"        # IMU sensor
        ]
        
        gazebo_plugins_path = "/opt/ros/humble/lib"
        
        for plugin in sensor_plugins:
            plugin_path = f"{gazebo_plugins_path}/{plugin}"
            exists = os.path.exists(plugin_path)
            self.print_status(f"Sensor Plugin: {plugin}", exists)
            self.results['sensors'][plugin] = exists

    def check_workspace_setup(self):
        """Check if user has a proper ROS2 workspace"""
        self.print_header("Workspace Configuration")
        
        # Common workspace locations
        workspace_paths = [
            "~/ros2_ws",
            "~/workspace", 
            "~/dev_ws",
            "~/robot_ws"
        ]
        
        workspace_found = False
        for ws_path in workspace_paths:
            expanded_path = os.path.expanduser(ws_path)
            src_path = os.path.join(expanded_path, "src")
            install_path = os.path.join(expanded_path, "install")
            
            if os.path.exists(src_path):
                workspace_found = True
                has_install = os.path.exists(install_path)
                self.print_status(f"Workspace found: {ws_path}", True)
                self.print_status(f"  Install directory", has_install)
                
                # Check for proto3 packages
                proto3_desc = os.path.exists(os.path.join(src_path, "proto3_description"))
                proto3_ctrl = os.path.exists(os.path.join(src_path, "proto3_controller"))
                
                if proto3_desc:
                    self.print_status("  proto3_description package", True)
                if proto3_ctrl:
                    self.print_status("  proto3_controller package", True)
                
                break
        
        if not workspace_found:
            self.print_status("ROS2 workspace found", False, "Consider creating ~/ros2_ws")
        
        self.results['environment']['workspace_found'] = workspace_found

    def test_basic_functionality(self):
        """Test basic ROS2 functionality"""
        self.print_header("Basic Functionality Tests")
        
        # Test ros2 topic list
        success, stdout, stderr = self.run_command("timeout 5 ros2 topic list", timeout=10)
        self.print_status("ros2 topic list", success)
        
        # Test ros2 node list  
        success, stdout, stderr = self.run_command("timeout 5 ros2 node list", timeout=10)
        self.print_status("ros2 node list", success)
        
        # Test ros2 service list
        success, stdout, stderr = self.run_command("timeout 5 ros2 service list", timeout=10)
        self.print_status("ros2 service list", success)

    def generate_recommendations(self):
        """Generate recommendations based on verification results"""
        self.print_header("Recommendations and Next Steps")
        
        issues = []
        
        # Check for critical issues
        if self.results['environment']['ros_distro'] != 'humble':
            issues.append("ROS2 Humble not properly sourced")
        
        # Check package installations
        missing_packages = [pkg for pkg, installed in self.results['packages'].items() if not installed]
        if missing_packages:
            issues.append(f"Missing packages: {', '.join(missing_packages[:5])}")
        
        # Check Gazebo plugins
        if not self.results['gazebo'].get('ros2_control_plugin', False):
            issues.append("gazebo_ros2_control plugin not found")
        
        if issues:
            print(f"{Colors.YELLOW}⚠ Issues found:{Colors.RESET}")
            for i, issue in enumerate(issues, 1):
                print(f"{Colors.YELLOW}{i}. {issue}{Colors.RESET}")
                
            print(f"\n{Colors.CYAN}Recommended fixes:{Colors.RESET}")
            print(f"{Colors.WHITE}1. Source ROS2 Humble:{Colors.RESET}")
            print(f"   source /opt/ros/humble/setup.bash")
            
            if missing_packages:
                print(f"{Colors.WHITE}2. Install missing packages:{Colors.RESET}")
                print(f"   sudo apt install ros-humble-<package-name>")
                
            print(f"{Colors.WHITE}3. Rebuild your workspace:{Colors.RESET}")
            print(f"   cd ~/ros2_ws && colcon build")
        else:
            print(f"{Colors.GREEN}🎉 All systems appear to be working correctly!{Colors.RESET}")
            print(f"{Colors.GREEN}Your ROS2 Humble environment is ready for robotics development.{Colors.RESET}")

    def save_results(self, filename="ros2_humble_verification_results.json"):
        """Save verification results to file"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.results, f, indent=2)
            print(f"\n{Colors.CYAN}Results saved to: {filename}{Colors.RESET}")
        except Exception as e:
            print(f"\n{Colors.RED}Failed to save results: {e}{Colors.RESET}")

    def run_full_verification(self):
        """Run complete verification suite"""
        print(f"{Colors.BOLD}{Colors.MAGENTA}")
        print("="*70)
        print("ROS2 HUMBLE ENVIRONMENT VERIFICATION")
        print("Proto3 Robot Project - Synthetic Data Pipeline")
        print("="*70)
        print(f"{Colors.RESET}")
        
        try:
            self.check_environment()
            self.check_core_packages()
            self.check_gazebo_plugins()
            self.check_hardware_interfaces()
            self.check_controllers()
            self.check_moveit_setup()
            self.check_sensor_plugins()
            self.check_workspace_setup()
            self.test_basic_functionality()
            self.generate_recommendations()
            self.save_results()
            
        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}Verification interrupted by user{Colors.RESET}")
        except Exception as e:
            print(f"\n{Colors.RED}Verification failed: {e}{Colors.RESET}")

def main():
    """Main function"""
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print(__doc__)
        return
    
    verifier = ROS2HumbleVerifier()
    verifier.run_full_verification()

if __name__ == "__main__":
    main()