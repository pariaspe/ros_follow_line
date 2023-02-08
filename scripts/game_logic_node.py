#!/usr/bin/python3
"""
game_logic_node.py
"""

import time
from math import isclose

import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import TriggerRequest, TriggerResponse, Trigger
from geometry_msgs.msg import Point

from tools import _

UPPER_X = 55.0
LOWER_X = 52.0
UPPER_Y = -10.7
LOWER_Y = -11.0

states = ["IDLE", "STARTED", "RUNNING"]


class GameLogicNode:
    def __init__(self, standalone: bool = False, verbose: bool = False) -> None:
        if standalone:
            log_level = rospy.DEBUG if verbose else rospy.INFO
            rospy.init_node('game_logic', log_level=log_level)

        self.state = states[0]
        self.init_time = None

        self.f1_pose = Point()

        rospy.Subscriber('F1ROS/odom', Odometry, self.odom_callback)
        rospy.Service('game_logic/start', Trigger, self.start_callback)
        rospy.Service('game_logic/stop', Trigger, self.stop_callback)

        rospy.loginfo("Game Logic Node started!")

    def spin(self) -> None:
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()

    def odom_callback(self, msg: Odometry) -> None:
        self.f1_pose = msg.pose.pose.position
        if self.state == states[1] and self.is_running_point(self.f1_pose):
            self.state = states[2]  # RUNNING
            self.init_time = time.time()

        if self.state == states[2]:
            rospy.logdebug_throttle(5,
                                    f"{time.time() - self.init_time} - {self.f1_pose}")
            if self.is_running_point(self.f1_pose):
                if (time.time() - self.init_time) > 10:
                    rospy.loginfo_throttle(10,
                                           f"LAP TIME: {time.time() - self.init_time}")
                    rospy.loginfo_throttle(10,
                                           f"HASH CODE: {_(time.time() - self.init_time)}")
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

    def start_callback(self, req: TriggerRequest) -> TriggerResponse:
        """Start callback
        """
        if self.is_start_point(self.f1_pose):
            rospy.loginfo("STARTED!")
            self.state = states[1]  # STARTED
            return TriggerResponse(True, "")
        rospy.logwarn("F1 out of initial pose!")
        return TriggerResponse(False, "F1 out of initial pose!")

    def stop_callback(self, req: TriggerRequest) -> TriggerResponse:
        """Stop callback
        """
        rospy.loginfo("STOPPED!")
        self.state = states[0]  # IDLE
        self.init_time = None
        return TriggerResponse(True, "")


if __name__ == "__main__":
    try:
        game_logic = GameLogicNode(standalone=True, verbose=True)
        game_logic.spin()
    except rospy.ROSInterruptException:
        pass
