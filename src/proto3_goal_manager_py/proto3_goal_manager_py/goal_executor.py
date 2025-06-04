#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from proto3_interfaces.action import MoveToPose

from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

class GoalExecutor(Node):
  def __init__(self):
    super().__init__("goal_executor_server")

    self.motve_to_pose_action_server_ = ActionServer(self,
                                                     MoveToPose,
                                                     "move_to_pose",
                                                     self.execute_callback)
    self.get_logger().info("Move to Pose Action Server has started...")


  def execute_callback(self, goal_handle: ServerGoalHandle):
    target_pose = goal_handle.request.target_pose

    self.get_logger().info(f"Received goal with target pose: {target_pose}")
    time.sleep(2)
    goal_handle.succeed()
    result = MoveToPose.Result()

    result.success = True
    result.reached_pose = target_pose

    return result

    pass

def main(args=None):
  rclpy.init(args=args)
  node = GoalExecutor()
  rclpy.spin(node=node)
  node.destroy_node()
  rclpy.shutdown()

if __name__== "__main__":
  main()