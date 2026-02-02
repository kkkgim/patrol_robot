#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from patrol_control.robot_types import RobotState, RobotCmd

class ExecutorNode(Node):
    """
    실행 전용 노드
    - control_node 명령 수행
    - Nav2 실제 호출
    """

    def __init__(self):
        super().__init__('executor_node')

        self._status = RobotState.IDLE

        # ===============================
        # Nav2 Navigator
        # ===============================
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active')

        # ===============================
        # Subscriptions
        # ===============================
        self.cb_group = ReentrantCallbackGroup()
        self.create_subscription(
                String,
                '/control/command',
                self.command_cb,
                10,
                callback_group=self.cb_group  # 이 그룹에 속한 콜백은 병렬 실행 가능
            )
        # ===============================
        # Publishers
        # ===============================
        self.status_pub = self.create_publisher(String, '/executor/status', 10)

        # ===============================
        # Waypoints
        # ===============================
        self.waypoints = self._load_waypoints()

        self.get_logger().info('Executor Node started')


# ===============================
    # State Property (수정됨)
    # ===============================
    @property
    def status(self):
        return self._status  

    @status.setter
    def status(self, new_status: RobotState):
        if self._status != new_status: 
            old_status = self._status
            self._status = new_status  # 실제 값 업데이트
            
            # 퍼블리셔가 존재할 때만 발행
            if hasattr(self, 'status_pub'):
                msg = String(data=self._status.value)
                self.status_pub.publish(msg)
            
            self.get_logger().info(f'State change: {old_status.name} → {new_status.name}')


    # ======================================================
    # Command Callback
    # ======================================================
    def command_cb(self, msg: String):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        if command == RobotCmd.START_PATROL.to_string():
            self._start_patrol()

        elif command == RobotCmd.RETURN_HOME.to_string():
            self._return_home()

        elif command == RobotCmd.STOP.to_string():
            self._stop()

    # ======================================================
    # Execution Logic
    # ======================================================
    def _start_patrol(self):
            self.status=RobotState.PATROL_STARTED

            self.get_logger().info(f'start patrol: remain {len(self.waypoints)} points ')
            self.navigator.followWaypoints(self.waypoints)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    current = feedback.current_waypoint + 1
                    total = len(self.waypoints)
                    
                    self.get_logger().info(
                        f'>>> patrol .. [{current}/{total}] points'
                    )

            result = self.navigator.getResult()
                        
            if result == TaskResult.SUCCEEDED:
                        self.get_logger().info('Patrol complete')
                        self.status=RobotState.COMPLETED
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Patrol canceled')
                self.status=RobotState.STOPPED
            else:
                self.get_logger().error('Patrol error, stopped patrol')
                self.status=RobotState.FAILED

    def _return_home(self):
        self._cancel_if_active()
        self.status=RobotState.RETURN_HOME

        home_pose = self.get_position('map', 0.0, 0.0)
        self.navigator.goToPose(home_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            
            if feedback:
                # 남은 거리 추출 (미터 단위)
                dist = feedback.distance_remaining
                self.get_logger().info(f'>>> return... remain distance: {dist:.2f}m')
            
            time.sleep(1.0) # 로그 너무 많이 찍히지 않게 1초 간격


        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Return complete')
            self.status=RobotState.COMPLETED
        

    def _stop(self):
        self._cancel_if_active()
        self.status=RobotState.STOPPED


    # ======================================================
    # Utilities
    # ======================================================
    def _cancel_if_active(self):
        self.navigator.cancelTask()

    def _load_waypoints(self):
        waypoints = []

        coords = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]

        for x, y in coords:
            pose = self.get_position('map', x, y)
            waypoints.append(pose)

        return waypoints
    
    def get_position(self, frame_id, x, y):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        return pose

def main(args=None):
    rclpy.init(args=args)
    node = ExecutorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin() 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
