import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        self.get_logger().info("Navigator has been started")

        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.orientation.w = 1.0

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.target_pose

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for the navigation server')

        self.get_logger().info("Sending goal request...")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result():
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation succeeded!')
            else:
                self.get_logger().warn(
                    'Navigation failed with status: {0}'.format(status))

        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        distance_remaining = feedback.distance_remaining
        self.get_logger().info(
            'Distance remaining: {:.2f}'.format(distance_remaining))


def main(args=None):
    rclpy.init(args=args)

    action_client_node = Navigator()
    action_client_node.send_goal()

    rclpy.spin(action_client_node)


if __name__ == '__main__':
    main()
