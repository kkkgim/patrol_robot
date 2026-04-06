#!/usr/bin/env python3
"""
Control Node - 순찰 로봇 제어 노드
"""
import rclpy
import yaml
import os
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rcl_interfaces.msg import SetParametersResult

CONFIG_PATH = os.environ.get(
    'CONFIG_PATH',
    '/root/patrol_robot/src/patrol_control/config/operation_policy.yaml'
)

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # 파라미터 선언 (기본값은 코드 fallback용)
        self.declare_parameter('operation_start', '22:00')
        self.declare_parameter('operation_end', '06:00')
        self.declare_parameter('auto_patrol', True)

        self.last_command = None

        # config 초기값 로드
        self._load_config_from_yaml()

        # 현재 파라미터 값을 멤버변수에 적용
        self.apply_parameters()

        # 웹서버에서 set_parameters 호출 시 콜백
        self.add_on_set_parameters_callback(self.on_param_change)

        # Publisher
        self.command_pub = self.create_publisher(String, '/control/command', 10)

        # 서비스 서버
        self.start_srv = self.create_service(
            Trigger,
            '/control/start_patrol',
            self.start_patrol_cb
        )
        self.home_srv = self.create_service(
            Trigger,
            '/control/return_home',
            self.return_home_cb
        )
        self.stop_srv = self.create_service(
            Trigger,
            '/control/stop',
            self.stop_cb
        )

        # 타이머 (자동 순찰 체크 - 10초마다)
        self.timer = self.create_timer(10.0, self.timer_cb)

        self.create_subscription(String,'/executor/status',self.status_cb,1)
        self.get_logger().info('Control Node started')

    # -------------------------------------------------------
    # Config 로드
    # -------------------------------------------------------
    def _load_config_from_yaml(self):
        """YAML 파일에서 운영 설정 읽어서 파라미터에 반영"""
        try:
            with open(CONFIG_PATH, 'r') as f:
                data = yaml.safe_load(f)

            # operation_policy.yaml 구조:
            ros_params = (
                data.get('control_node', {})
                    .get('ros__parameters', {})
            )

            if not ros_params:
                self.get_logger().warn('[Config] ros__parameters 없음, 기본값 사용')
                return

            # 파라미터 덮어쓰기
            for key, value in ros_params.items():
                self.set_parameters([
                    rclpy.parameter.Parameter(key, value=value)
                ])

            self.get_logger().info(f'[Config] YAML 로드 완료: {ros_params}')

        except FileNotFoundError:
            self.get_logger().warn(f'[Config] 파일 없음: {CONFIG_PATH}, 기본값 사용')
        except Exception as e:
            self.get_logger().error(f'[Config] 로드 실패: {e}, 기본값 사용')

    # -------------------------------------------------------
    # 파라미터 적용 / 변경 콜백
    # -------------------------------------------------------
    def apply_parameters(self):
        self.operation_start = self.get_parameter('operation_start').value
        self.operation_end   = self.get_parameter('operation_end').value
        self.auto_patrol     = self.get_parameter('auto_patrol').value
        self.get_logger().info(
            f'[Params] start={self.operation_start}, '
            f'end={self.operation_end}, auto_patrol={self.auto_patrol}'
        )

    def on_param_change(self, params):
        """웹서버에서 set_parameters 호출 시 자동 실행"""

        for p in params:
            self.get_logger().info(f'[Param 변경] {p.name} = {p.value}')
            if hasattr(self, p.name):
                        setattr(self, p.name, p.value)
        
        return SetParametersResult(successful=True)

    # -------------------------------------------------------
    # 타이머
    # -------------------------------------------------------
    def timer_cb(self):
        """자동 순찰 타이머"""
        self.get_logger().info(f'[Auto Patrol Check] Auto Patrol Status: {self.auto_patrol}')
        if not self.auto_patrol:
            return
              
        # 1. ROS 2 시뮬레이션 시간 가져오기 (Time 객체)
        now_ros = self.get_clock().now()
        
        # 2. Time 객체를 초 단위로 변환 후 HH:MM 문자열 생성
        # nanoseconds를 초로 환산 (10^9로 나눔)
        total_seconds = now_ros.nanoseconds / 1e9
        m, _ = divmod(int(total_seconds), 60)
        h, m = divmod(m, 60)
        
        # 시뮬레이션 타임이 24시간을 넘을 수 있으므로 % 24 처리
        now_str = f"{h % 24:02d}:{m:02d}"
        self.get_logger().info(f'[Timer Check] now: {now_str}')
        
        start = self.operation_start
        end   = self.operation_end

        if self.is_operation_time(now_str, start, end):
            self.get_logger().info(f'[Timer] 운영시간 [{now_str}] - 자동 순찰')
            self.send_command('START_PATROL')

    def is_operation_time(self, now: str, start: str, end: str):
        """운영시간 확인 (자정 넘는 경우 처리)"""
        if start < end:
            return start <= now <= end
        else:
            # 자정 넘음: 22:00 ~ 06:00
            return now >= start or now <= end

    # -------------------------------------------------------
    # 명령 전송
    # -------------------------------------------------------
    def send_command(self, command: str):
        msg = String()
        if self.last_command==command or self.last_command=='FAILED' :
            return
        
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'[Command] {command}')


    # -------------------------------------------------------
    # 서비스 콜백
    # -------------------------------------------------------
    def start_patrol_cb(self, request, response):
        self.get_logger().info('Service: START_PATROL')
        self.send_command('START_PATROL')
        response.success = True
        response.message = 'Patrol started'

        return response

    def return_home_cb(self, request, response):
        self.get_logger().info('Service: RETURN_HOME')
        self.send_command('RETURN_HOME')
        response.success = True
        response.message = 'Returning home'

        return response

    def stop_cb(self, request, response):
        self.get_logger().info('Service: STOP')
        self.send_command('STOP')
        response.success = True
        response.message = 'Stopped'

        return response
    
    # -------------------------------------------------------
    # 구독 콜백
    # -------------------------------------------------------
    def status_cb(self, msg: String):
        self.last_command = msg.data
        self.get_logger().info(f'Received status: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()