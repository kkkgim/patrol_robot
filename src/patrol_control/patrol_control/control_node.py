import rclpy
from rclpy.node import Node
from patrol_control.robot_types import RobotState, STATE_COMMAND_MAP
from datetime import datetime, time

from std_msgs.msg import String
from std_srvs.srv import Trigger

class ControlNode(Node):
    """
    FSM 기반 중앙 제어 노드
    - 상태 관리 (RobotState)
    - 정책 판단
    - executor에 명령 전달
    """

    def __init__(self):
        super().__init__('control_node')

        # ===============================
        # FSM State
        # ===============================
        self._state: RobotState = RobotState.IDLE

        # ===============================
        # Parameters (운영 정책)
        # ===============================
        self.declare_parameter('operation_start', '22:00')
        self.declare_parameter('operation_end', '06:00')
        self.declare_parameter('auto_patrol', True)

        # ===============================
        # Services (from FMS / UI)
        # ===============================
        self.create_service(Trigger, '/control/start_patrol', self.start_patrol_cb)
        self.create_service(Trigger, '/control/stop', self.stop_cb)
        self.create_service(Trigger, '/control/return_home', self.return_home_cb)

        # ===============================
        # Publishers
        # ===============================
        self.state_pub = self.create_publisher(String, '/control/state', 10)
        self.command_pub = self.create_publisher(String, '/control/command', 10)

        # ===============================
        # Subscriptions
        # ===============================
        self.create_subscription(String,'/executor/status', self.executor_status_cb, 10)

        # ===============================
        # Timer
        # ===============================
        self.timer = self.create_timer(1.0, self.timer_cb)

        self.get_logger().info('Control Node started')


    # ===============================
    # State Property 
    # ===============================
    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state: RobotState):
        if self._state != new_state:
            old_state = self._state
            self._state = new_state
            
            # 상태가 바뀔 때 자동으로 publish
            msg = String(data=self._state.value)
            self.state_pub.publish(msg)
            
            self.get_logger().info(f'Robot state: {old_state.name} → {new_state.name}')

    # ======================================================
    # Timer
    # ======================================================
    def timer_cb(self):
        # 자동 순찰 정책
        if not self.get_parameter('auto_patrol').value:
            return

        if self._is_in_operation_time():
            self.get_logger().info('Operation time reached → auto patrol')
            if self.state in [RobotState.PATROL_STARTED, RobotState.RETURN_HOME]:
                self.get_logger().info(f'Cannot start patrol from {self.state.value}')
                return
            self._transition(RobotState.PATROL_STARTED)

    # ======================================================
    # Service Callbacks
    # ======================================================
    def start_patrol_cb(self, request, response):
        if self.state in [RobotState.FAILED, RobotState.PATROL_STARTED]:
            response.success = False
            response.message = f'Cannot start patrol from {self.state.value}'
            return response

        self._transition(RobotState.PATROL_STARTED)
        response.success = True
        response.message = 'Patrol started'
        return response

    def stop_cb(self, request, response):
        if self.state not in [RobotState.PATROL_STARTED, RobotState.RETURN_HOME]:
            response.success = False
            response.message = f'Cannot stop patrol from {self.state.value}'
            return response

        self._transition(RobotState.STOPPED)
        response.success = True
        response.message = 'Robot stopped'
        return response

    def return_home_cb(self, request, response):
        if self.state in [RobotState.FAILED, RobotState.RETURN_HOME]:
            response.success = False
            response.message = f'Cannot return home from {self.state.value}'
            return response
        
        self._transition(RobotState.RETURN_HOME)
        
        response.success = True
        response.message = 'Returning home'
        return response
    
    # ======================================================
    # Subscriptions Callbacks
    # ======================================================
    def executor_status_cb(self, msg: String):
        try:
            incoming_state = RobotState(msg.data)
        except ValueError:
            self.get_logger().warn(f'Unknown executor status: {msg.data}')
            return

        # if incoming_state in [RobotState.COMPLETED, RobotState.STOPPED]:
        #     self.get_logger().info(f'{incoming_state} → {RobotState.IDLE}')
        #     self.state = RobotState.IDLE
        if incoming_state == RobotState.FAILED:
            self.state = RobotState.FAILED

    # ======================================================
    # Transitions
    # ======================================================
    def _transition(self, next_state: RobotState):
        cmd = STATE_COMMAND_MAP[next_state]

        self.get_logger().info(f'{self.state} → {next_state}')
        self.state = next_state
        self.command_pub.publish(String(data=cmd.value))

    # ======================================================
    # Policy Helpers
    # ======================================================
    def _is_in_operation_time(self) -> bool:
        start_str = self.get_parameter('operation_start').value
        end_str = self.get_parameter('operation_end').value

        start = self._parse_time(start_str)
        end = self._parse_time(end_str)
        now = datetime.now().time()

        # 자정 안 넘는 경우 (09:00 ~ 18:00)
        if start < end:
            return start <= now <= end

        # 자정 넘는 경우 (22:00 ~ 06:00)
        return now >= start or now <= end

    def _parse_time(self, t: str) -> time:
        h, m = map(int, t.split(':'))
        return time(hour=h, minute=m)
    

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
 