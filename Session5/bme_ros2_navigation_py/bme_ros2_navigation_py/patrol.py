import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus

from tf_transformations import quaternion_from_euler


class PatrolNode(Node):

    def __init__(self):
        super().__init__("patrol_node")

        self.action_client = ActionClient(
            self,
            FollowWaypoints,
            "follow_waypoints"
        )

        self.get_logger().info("Waiting for FollowWaypoints server...")
        self.action_client.wait_for_server()
        self.get_logger().info("Connected to FollowWaypoints server!")

        self.send_goal()

    def create_pose(self, x, y, theta):

        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, theta)

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def send_goal(self):

        goal_msg = FollowWaypoints.Goal()

        waypoints = [
            (-1.0, -1.0, 0.0),
            (1.0, -1.0, 0.0),
            (1.0, 1.0, 0.0),
            (-1.0, 0.0, 0.0),
        ]

        for x, y, theta in waypoints:
            goal_msg.poses.append(
                self.create_pose(x, y, theta)
            )

        self.get_logger().info("Sending patrol...")

        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):

        current = feedback_msg.feedback.current_waypoint

        self.get_logger().info(
            f"Navigating to Waypoint {current + 1}"
        )

    def result_callback(self, future):

        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Patrol completed successfully!")
        else:
            self.get_logger().info(
                f"Patrol failed. Status: {status}"
            )

        self.destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = PatrolNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()