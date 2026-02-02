#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

from datetime import datetime, time


class ControlNode(Node):
    """
    중앙 관리자 노드
    
    """

    #---- 초기화 시작 ----#
    def __init__(self):
        super().__init__('control_node')

        # ===============================
        # Global Control State
        # ===============================
        self.state = 'IDLE'   # IDLE, PATROL, RETURN_HOME, STOPPED
        self.current_mode = None

        # ===============================
        # Nav2 Navigator
        # ===============================
        self.navigator = BasicNavigator()

        # ===============================
        # Parameters
        # ===============================

        # self.declare_parameter('operation_start', '22:00')
        # self.declare_parameter('operation_end', '06:00')

        # self.operation_start = self._parse_time(
        #     self.get_parameter('operation_start').value)
        # self.operation_end = self._parse_time(
        #     self.get_parameter('operation_end').value)

        self.declare_parameter('patrol_timeout_sec', 3600)

        self.patrol_timeout = Duration(seconds=self.get_parameter('patrol_timeout_sec').value)

        # ===============================
        # Time Tracking
        # ===============================
        self.mode_start_time = None

        # ===============================
        # User Control Services
        # ===============================
        self.create_service(
            Trigger, '/control/start_patrol', self.start_patrol_cb)

        self.create_service(
            Trigger, '/control/stop', self.stop_cb)

        self.create_service(
            Trigger, '/control/return_home', self.return_home_cb)

        # ===============================
        # Status Publishers
        # ===============================
        self.state_pub = self.create_publisher(
            String, '/control/state', 10)

        self.operation_pub = self.create_publisher(
            Bool, '/control/operation_allow', 10)

        # ===============================
        # Main Control Timer
        # ===============================
        self.timer = self.create_timer(1.0, self.timer_cb)

        self.get_logger().info('Control Node started')

    #---- 초기화 종료 ----#
    

    # ======================================================
    # Utility
    # ======================================================
    # def _parse_time(self, time_str):
    #     h, m = map(int, time_str.split(':'))
    #     return time(hour=h, minute=m)

    # def is_operation_time(self):
    #     now = datetime.now().time()

    #     # operation time (22:00 ~ 06:00)
    #     if self.operation_start <= self.operation_end:
    #         return self.operation_start <= now <= self.operation_end
    #     else:
    #         return now >= self.operation_start or now <= self.operation_end

    # ======================================================
    # Timer Callback (Policy + Safety)
    # ======================================================
    def timer_cb(self):
        # Publish current policy state
        self.operation_pub.publish(Bool(data=self.is_operation_time()))
        self.state_pub.publish(String(data=self.state))

        # Patrol timeout check
        if self.state == 'PATROL' and self.mode_start_time:
            elapsed = self.get_clock().now() - self.mode_start_time
            if elapsed > self.patrol_timeout:
                self.get_logger().warn('Patrol timeout')
                self._cancel_and_return_home()

        # Operation allow closed
        # if self.state == 'PATROL' and not self.is_operation_time():
        #     self.get_logger().info('Operation allow closed')
        #     self._cancel_and_return_home()

    # ======================================================
    # User Service Callbacks
    # ======================================================
    def start_patrol_cb(self, request, response):
        if not self.is_operation_time():
            response.success = False
            response.message = 'Outside operation time'
            return response

        if self.state != 'IDLE':
            response.success = False
            response.message = 'Robot is busy'
            return response

        self._enter_patrol_mode()
        response.success = True
        response.message = 'Patrol mode started'
        return response

    def stop_cb(self, request, response):
        self._cancel_all()
        self.state = 'STOPPED'
        self.current_mode = None
        response.success = True
        response.message = 'Robot stopped'
        return response

    def return_home_cb(self, request, response):
        self._cancel_and_return_home()
        response.success = True
        response.message = 'Returning home'
        return response

    # ======================================================
    # Mode Control Logic
    # ======================================================
    def _enter_patrol_mode(self):
        self.state = 'PATROL'
        self.current_mode = 'PATROL'
        self.mode_start_time = self.get_clock().now()

        self.get_logger().info('Entering PATROL mode')

        # TODO: delegate to patrol_executor
        # self.patrol_executor.start()

    def _cancel_all(self):
        if self.navigator.isTaskActive():
            self.navigator.cancelTask()

    def _cancel_and_return_home(self):
        self._cancel_all()
        self._enter_return_home_mode()

    def _enter_return_home_mode(self):
        self.state = 'RETURN_HOME'
        self.current_mode = 'RETURN_HOME'
        self.get_logger().info('Entering RETURN_HOME mode')

        home_pose = self._get_home_pose()
        self.navigator.goToPose(home_pose)

    # ======================================================
    # Home Pose
    # ======================================================
    def _get_home_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
