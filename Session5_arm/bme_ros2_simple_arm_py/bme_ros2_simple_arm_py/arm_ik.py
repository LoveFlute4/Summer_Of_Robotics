#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from bme_ros2_simple_arm_py.test_inverse_kinematics import inverse_kinematics


class ArmIK(Node):

    def __init__(self):

        super().__init__("arm_ik")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.get_logger().info("Waiting for arm controller...")

        self.client.wait_for_server()

        self.get_logger().info("Controller connected!")

        # ------------------------------
        # CHANGE THIS TARGET AS REQUIRED
        # ------------------------------

        target = [0.40, 0.10, 0.15]

        joint_angles = inverse_kinematics(
            target,
            "open",
            0.0
        )

        self.send_goal(joint_angles[:4])

    def send_goal(self, positions):

        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint"
        ]

        point = JointTrajectoryPoint()

        point.positions = positions
        point.time_from_start.sec = 3

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(
            f"Sending goal:\n{[round(p,3) for p in positions]}"
        )

        self.send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self.send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):
        pass

    def result_callback(self, future):

        result = future.result().result
        error_code = result.error_code

        if error_code == 0:
            self.get_logger().info("Motion completed successfully.")
        else:
            self.get_logger().error(
                f"Controller returned error code: {error_code}"
            )

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = ArmIK()

    rclpy.spin(node)


if __name__ == "__main__":
    main()