#!/usr/bin/env python3
"""
Executor Node - 순찰 로봇 실행 노드
- command_cb: 즉시 리턴 (블로킹 금지)
- _execute_command: 별도 스레드에서 Nav2 호출 (time.sleep OK)
- 웨이포인트: operation_policy.yaml에서 로드
- 웨이포인트: /patrol/waypoints 토픽으로 퍼블리시 (웹 시각화)
"""

import rclpy
import time
import threading
import yaml
import os

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from patrol_control.robot_types import RobotState, RobotCmd
from typing import Optional


CONFIG_PATH = os.environ.get(
    'CONFIG_PATH',
    '/root/patrol_robot/src/patrol_control/config/operation_policy.yaml'
)

class ExecutorNode(Node):

    CANCEL_TIMEOUT_SEC        = 3.0
    FEEDBACK_LOG_INTERVAL_SEC = 1.0
    TASK_POLL_INTERVAL_SEC    = 0.1

    def __init__(self):
        super().__init__('executor_node')

        self._status = RobotState.IDLE

        # 실행 스레드 관리
        self._exec_thread: Optional[threading.Thread] = None
        self._exec_lock   = threading.Lock()

        # ── Nav2 ──
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active')

        # ── Subscriber ──
        self.cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            String, '/control/command', self.command_cb, 1,
            callback_group=self.cb_group
        )

        # ── Publisher: 상태 ──
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.status_pub = self.create_publisher(
            String, '/executor/status', qos_profile=status_qos
        )

        self.get_logger().info('Executor Node started')

    # =====================================================
    # State Property
    # =====================================================
    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, new_status: RobotState):
        if self._status != new_status:
            old = self._status
            self._status = new_status
            if hasattr(self, 'status_pub'):
                self.status_pub.publish(String(data=self._status.value))
            self.get_logger().info(f'State: {old.name} → {new_status.name}')

    # =====================================================
    # Waypoints
    # =====================================================
    def _load_waypoints(self):
        try:
            with open(CONFIG_PATH, 'r') as f:
                data = yaml.safe_load(f)
            raw = data.get('waypoints', [])
            if not raw:
                self.get_logger().warn('[Waypoints] config에 없음, 기본값 사용')
                return self._default_waypoints()
            waypoints = [self.get_position('map', float(wp[0]), float(wp[1])) for wp in raw]
            
            return waypoints
    
        except FileNotFoundError:
            self.get_logger().warn(f'[Waypoints] 파일 없음: {CONFIG_PATH}')
            return self._default_waypoints()
        except Exception as e:
            self.get_logger().error(f'[Waypoints] 로드 실패: {e}')
            return self._default_waypoints()
        
    def _load_homepoint(self):
        try:
            with open(CONFIG_PATH, 'r') as f:
                data = yaml.safe_load(f)
            raw = data.get('homepoint', [])
            if not raw:
                self.get_logger().warn('[Homepoint] config에 없음, 기본값 사용')
                return self._default_homepoint()

            x, y = raw[0]
            return self.get_position('map', float(x), float(y))
            
        except FileNotFoundError:
            self.get_logger().warn(f'[Waypoints] 파일 없음: {CONFIG_PATH}')
            return self._default_homepoint()
        except Exception as e:
            self.get_logger().error(f'[Waypoints] 로드 실패: {e}')
            return self._default_homepoint()

    def _default_waypoints(self):
        coords = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        return [self.get_position('map', x, y) for x, y in coords]
    
    def _default_homepoint(self):
        return self.get_position('map', 0.0, 0.0)

    # =====================================================
    #   Command Callback - 즉시 리턴 (블로킹 절대 금지)
    #   spin_once 호출 금지 → MultiThreadedExecutor와 충돌
    # =====================================================
    def command_cb(self, msg: String):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # 별도 스레드로 실행 → ROS2 spin 간섭 없음
        with self._exec_lock:
            self._exec_thread = threading.Thread(
                target=self._execute_command,
                args=(command,),
                daemon=True
            )
            self._exec_thread.start()

    # =====================================================
    # 실행 로직 (별도 스레드 - time.sleep 사용 가능)
    # =====================================================
    def _execute_command(self, command: str):
        if command == RobotCmd.START_PATROL.to_string():
            self._start_patrol()
        elif command == RobotCmd.RETURN_HOME.to_string():
            self._return_home()
        elif command == RobotCmd.STOP.to_string():
            self._stop()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def _start_patrol(self):
        self._cancel_if_active()
        self.status = RobotState.START_PATROL

        self.waypoints = self._load_waypoints()
        self.get_logger().info(f'Start patrol: {len(self.waypoints)} waypoints')

        self.navigator.followWaypoints(self.waypoints)
        result = self._wait_for_task_completion(self._log_patrol_progress)
        self._handle_task_result(result, 'Patrol')

    def _return_home(self):
        self._cancel_if_active()
        self.status = RobotState.RETURN_HOME
        self.homepoints = self._load_homepoint()
        self.get_logger().info('Returning home...')

        self.navigator.goToPose(self.homepoints)
        result = self._wait_for_task_completion(self._log_return_progress)
        self._handle_task_result(result, 'Return home')

    def _stop(self):
        self._cancel_if_active()
        self.get_logger().info('Stopping robot')
        self.status = RobotState.STOP

    # =====================================================
    # Utilities (별도 스레드 - time.sleep OK)
    # =====================================================
    def _wait_for_task_completion(self, feedback_callback=None) -> TaskResult:
        last_log_time = self.get_clock().now()

        while not self.navigator.isTaskComplete():
            if self._is_task_cancelled():
                self.get_logger().info('Task cancelled externally')
                return TaskResult.CANCELED

            feedback = self.navigator.getFeedback()
            if feedback and feedback_callback:
                now = self.get_clock().now()
                if (now - last_log_time).nanoseconds / 1e9 >= self.FEEDBACK_LOG_INTERVAL_SEC:
                    feedback_callback(feedback)
                    last_log_time = now

            time.sleep(self.TASK_POLL_INTERVAL_SEC)  # spin_once 대신

        return self.navigator.getResult()

    def _cancel_if_active(self) -> bool:
        if self.navigator.isTaskComplete():
            return True

        self.get_logger().info('Cancelling current task...')
        self.navigator.cancelTask()

        start_time = self.get_clock().now()
        timeout    = Duration(seconds=self.CANCEL_TIMEOUT_SEC)

        while not self.navigator.isTaskComplete():
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().error(f'Cancel timeout ({self.CANCEL_TIMEOUT_SEC}s)')
                return False
            time.sleep(self.TASK_POLL_INTERVAL_SEC)

        while not self._is_task_cancelled():
            time.sleep(self.TASK_POLL_INTERVAL_SEC)

        self.get_logger().info('Task cancelled successfully')
        return True

    def _handle_task_result(self, result: TaskResult, task_name: str):
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'{task_name} completed')
            self.status = RobotState.COMPLETED
        elif result == TaskResult.CANCELED:
            self.get_logger().info(f'{task_name} cancelled')
            self.status = RobotState.CANCELLED
        else:
            self.get_logger().error(f'{task_name} failed')
            self.status = RobotState.FAILED

    def _is_task_cancelled(self) -> bool:
        return self.status not in [RobotState.START_PATROL, RobotState.RETURN_HOME]

    def _log_patrol_progress(self, feedback):
        self.get_logger().info(
            f'Patrol [{feedback.current_waypoint + 1}/{len(self.waypoints)}]'
        )

    def _log_return_progress(self, feedback):
        self.get_logger().info(f'Return: {feedback.distance_remaining:.2f}m remaining')

    def get_position(self, frame_id: str, x: float, y: float) -> PoseStamped:
        pose                    = PoseStamped()
        pose.header.frame_id   = frame_id
        pose.header.stamp      = self.get_clock().now().to_msg()
        pose.pose.position.x   = x
        pose.pose.position.y   = y
        pose.pose.orientation.w = 1.0
        return pose


# =====================================================
# Main
# =====================================================
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