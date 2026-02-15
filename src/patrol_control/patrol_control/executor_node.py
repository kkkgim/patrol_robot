import rclpy
import time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from patrol_control.robot_types import RobotState, RobotCmd
from typing import Callable, Optional

class ExecutorNode(Node):
    """
    실행 전용 노드
    - control_node 명령 수행
    - Nav2 실제 호출
    """
    
    # 상수 정의
    CANCEL_TIMEOUT_SEC = 3.0
    FEEDBACK_LOG_INTERVAL_SEC = 1.0
    TASK_POLL_INTERVAL_SEC = 0.1

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
                callback_group=self.cb_group
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
    # State Property
    # ===============================
    @property
    def status(self):
        return self._status  

    @status.setter
    def status(self, new_status: RobotState):
        if self._status != new_status: 
            old_status = self._status
            self._status = new_status
            
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
        """순찰 시작"""
        self._cancel_if_active()

        self.status = RobotState.PATROL_STARTED

        self.get_logger().info(f'Start patrol: {len(self.waypoints)} waypoints')
        self.navigator.followWaypoints(self.waypoints)

        # 순찰 완료 대기
        result = self._wait_for_task_completion(
            feedback_callback=self._log_patrol_progress
        )
        
        # 작업 결과 처리
        self._handle_task_result(result, "Patrol")


    def _return_home(self):
        """홈으로 복귀"""
        self._cancel_if_active()
        self.status = RobotState.RETURN_HOME

        home_pose = self.get_position('map', 0.0, 0.0)
        self.navigator.goToPose(home_pose)
        
        self.get_logger().info('Returning home...')
        
        # 귀환 완료 대기
        result = self._wait_for_task_completion(
            feedback_callback=self._log_return_progress
        )
        
        # 작업 결과 처리
        self._handle_task_result(result, "Return home")

        
    def _stop(self):
        """로봇 정지"""
        self._cancel_if_active()
        # 작업 결과 처리
        self.get_logger().info('Stopping robot')
        self.status = RobotState.STOPPED


    # ======================================================
    # Utilities
    # ======================================================
    def _wait_for_task_completion(self, feedback_callback: Optional[Callable] = None) -> TaskResult:
        """
        Args:
            feedback_callback: 피드백 로깅 함수
        Returns:
            TaskResult: 작업 결과
        """
        last_log_time = self.get_clock().now()
        
        while not self.navigator.isTaskComplete():
            # 중간에 취소되었는지 확인
            if self._is_task_cancelled():
                self.get_logger().info('Task was cancelled')
                return TaskResult.CANCELED
            
            # 피드백 처리 (로깅 간격 제어)
            feedback = self.navigator.getFeedback()
            if feedback and feedback_callback:
                current_time = self.get_clock().now()
                elapsed_sec = (current_time - last_log_time).nanoseconds / 1e9
                
                if elapsed_sec >= self.FEEDBACK_LOG_INTERVAL_SEC:
                    feedback_callback(feedback)
                    last_log_time = current_time
            
            time.sleep(self.TASK_POLL_INTERVAL_SEC)
        
        # 작업 완료 후 결과 반환
        return self.navigator.getResult()


    def _cancel_if_active(self) -> bool:
        """
        활성 작업이 있으면 취소
        
        Returns:
            bool: 취소 성공 여부
        """
        if self.navigator.isTaskComplete():
            return True
        
        self.get_logger().info('Cancelling current task...')
        self.navigator.cancelTask()
        
        start_time = self.get_clock().now()
        timeout = Duration(seconds=self.CANCEL_TIMEOUT_SEC)
        
        while not self.navigator.isTaskComplete():
            elapsed = self.get_clock().now() - start_time
            
            if elapsed > timeout:
                self.get_logger().error(
                    f'Cancel timeout ({self.CANCEL_TIMEOUT_SEC}s exceeded)'
                )
                return False
            
            time.sleep(self.TASK_POLL_INTERVAL_SEC)

        while not self._is_task_cancelled():
            time.sleep(self.TASK_POLL_INTERVAL_SEC)

        self.get_logger().info('Task cancelled successfully')

        return True


    def _handle_task_result(self, result: TaskResult, task_name: str):
        """
        작업 결과 처리
        
        Args:
            result: TaskResult
            task_name: 작업 이름 (로깅용)
        """
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'{task_name} completed successfully')
            self.status = RobotState.COMPLETED
            
        elif result == TaskResult.CANCELED:
            self.get_logger().info(f'{task_name} was cancelled')
            self.status = RobotState.CANCELLED
            
        else:
            self.get_logger().error(f'{task_name} failed')
            self.status = RobotState.FAILED


    def _is_task_cancelled(self) -> bool:
        """현재 작업이 취소되었는지 확인"""
        return self.status not in [
            RobotState.PATROL_STARTED,
            RobotState.RETURN_HOME
        ]


    def _log_patrol_progress(self, feedback):
        """순찰 진행 상황 로깅"""
        current = feedback.current_waypoint + 1
        total = len(self.waypoints)
        self.get_logger().info(f'Patrol progress: [{current}/{total}] waypoints')


    def _log_return_progress(self, feedback):
        """귀환 진행 상황 로깅"""
        dist = feedback.distance_remaining
        self.get_logger().info(f'Return home: {dist:.2f}m remaining')


    def _load_waypoints(self):
        """Waypoints 로드"""
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
        """포즈 생성"""
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