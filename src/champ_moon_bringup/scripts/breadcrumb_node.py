#!/usr/bin/env python3
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowWaypoints
from std_srvs.srv import Trigger


class BreadcrumbNode(Node):
    def __init__(self):
        super().__init__('breadcrumb_node')

        self.declare_parameter('breadcrumb_spacing', 0.5)
        self.spacing = self.get_parameter('breadcrumb_spacing').value

        # The trail itself - just a plain Python list of PoseStamped,
        # living in memory for the lifetime of this node. Nothing fancier.
        self.trail = []
        self.last_pose = None

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.path_pub = self.create_publisher(Path, '/breadcrumb_trail', 10)
        self.return_srv = self.create_service(
            Trigger, '/return_to_start', self.return_to_start_cb)
        self.follow_waypoints_client = ActionClient(
            self, FollowWaypoints, '/follow_waypoints')

        self.get_logger().info(
            f'Breadcrumb node started (spacing={self.spacing}m). '
            f'Call /return_to_start to retrace the recorded trail.')

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        if self.last_pose is None:
            self._drop_breadcrumb(pose)
            return

        dx = pose.pose.position.x - self.last_pose.pose.position.x
        dy = pose.pose.position.y - self.last_pose.pose.position.y
        if math.hypot(dx, dy) >= self.spacing:
            self._drop_breadcrumb(pose)

    def _drop_breadcrumb(self, pose: PoseStamped):
        self.trail.append(pose)
        self.last_pose = pose
        self._publish_trail()
        self.get_logger().info(
            f'Dropped breadcrumb #{len(self.trail)} at '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

    def _publish_trail(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = self.trail
        self.path_pub.publish(path)

    def return_to_start_cb(self, request, response):
        if len(self.trail) < 2:
            response.success = False
            response.message = 'Not enough breadcrumbs recorded yet.'
            return response

        if not self.follow_waypoints_client.wait_for_server(timeout_sec=3.0):
            response.success = False
            response.message = 'follow_waypoints action server not available.'
            return response

        # Reverse the trail to walk back the way we came. Skip the very
        # last entry - that's the robot's current position, not a waypoint
        # to drive to.
        waypoints = list(reversed(self.trail[:-1]))

        goal = FollowWaypoints.Goal()
        goal.poses = waypoints
        self.follow_waypoints_client.send_goal_async(goal)

        response.success = True
        response.message = f'Retracing {len(waypoints)} breadcrumbs back to start.'
        return response


def main():
    rclpy.init()
    node = BreadcrumbNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
