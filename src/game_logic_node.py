"""
game_logic_node.py
"""

import time
from math import isclose

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

UPPER_X = 55.0
LOWER_X = 52.0
UPPER_Y = -10.7
LOWER_Y = -11.0

states = ["IDLE", "STARTED", "RUNNING"]


class GameLogicNode(Node):
    """Game Logic Node
    """

    def __init__(self, verbose=False) -> None:
        super().__init__('game_logic', namespace='game_logic')
        if verbose:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.state = states[0]  # IDLE
        self.init_time = None

        self.f1_pose = Point()

        self.start_srv = self.create_service(Trigger, 'start', self.start)
        self.stop_srv = self.create_service(Trigger, 'stop', self.stop)

        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.get_logger().info("IDLE!")

    def start(self, request: Trigger.Request, response: Trigger.Response):
        """Start callback
        """
        if self.is_start_point(self.f1_pose):
            self.get_logger().info("STARTED!")
            self.state = states[1]  # STARTED
            response.success = True
            return response
        response.success = False
        response.message = "F1 out of initial pose!"
        self.get_logger().warn("F1 out of initial pose!")
        return response

    def stop(self, request: Trigger.Request, response: Trigger.Response):
        """Stop callback
        """
        self.get_logger().info("STOPPED!")
        self.state = states[0]  # IDLE
        self.init_time = None
        response.success = True
        return response

    def odom_callback(self, msg: Odometry) -> None:
        """Odometry callback
        """
        self.f1_pose = msg.pose.pose.position
        if self.state == states[1] and self.is_running_point(self.f1_pose):
            self.state = states[2]  # RUNNING
            self.init_time = time.time()

        if self.state == states[2]:
            self.get_logger().debug(
                f"{time.time() - self.init_time} - {self.f1_pose}", throttle_duration_sec=5)
            if self.is_running_point(self.f1_pose):
                if (time.time() - self.init_time) > 10:
                    self.get_logger().info(
                        f"LAP TIME: {time.time() - self.init_time}", throttle_duration_sec=10)
                self.init_time = time.time()  # new lap or reseting time if stopped

    def is_start_point(self, point: Point) -> bool:
        """Checks if the point is an valid position
        """
        if point.x > LOWER_X and point.x < UPPER_X:
            if point.y > LOWER_Y and point.y < UPPER_Y:
                return True
        return False

    def is_running_point(self, point: Point) -> bool:
        """Checks if the point is in a running point
        """
        if point.x > LOWER_X and point.x < UPPER_X:
            if isclose(point.y, LOWER_Y, rel_tol=0.01):
                return True
        return False


def main(args=None):
    """Main method
    """
    rclpy.init(args=args)

    node = GameLogicNode(verbose=True)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
