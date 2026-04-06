#!/usr/bin/env python3
"""
ROS2 노드 - 웹 API용
"""
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import base64
import numpy as np
from PIL import Image
import io
import asyncio
import time

class PatrolControlNode(Node):

    def __init__(self, websocket_manager):
        super().__init__('patrol_control_api')

        self.ws_manager = websocket_manager
        self.sim_time = None
        self._last_clock_broadcast = 0  # 스로틀용

        # 서비스 클라이언트
        self.cli_start = self.create_client(Trigger, '/control/start_patrol')
        self.cli_home  = self.create_client(Trigger, '/control/return_home')
        self.cli_stop  = self.create_client(Trigger, '/control/stop')

        # 파라미터 클라이언트
        self.cli_get_params = self.create_client(
            GetParameters, '/control_node/get_parameters'
        )
        self.cli_set_params = self.create_client(
            SetParameters, '/control_node/set_parameters'
        )

        '''
        durability - 내구성
        ㄴ TRANSIENT_LOCAL : 구독시 퍼블리셔로부터 마지막 데이터를 받음
        ㄴ VOLATILE : 구독을 시작한 그 순간부터 들어오는 데이터만 받음
        reliability - 신뢰성 
        ㄴ RELIABLE : 데이터가 유실되면 다시 보냄. 반드시 순서대로, 정확하게 도착하는 것을 보장
        ㄴ BEST_EFFORT : 유실되어도 다시 보내지 않음, 속도 중심
        '''
        
        # map은 Transient Local로 퍼블리시됨 - 늦게 구독해도 받을 수 있게
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
        status_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 구독
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos  
        )
        self.status_sub = self.create_subscription(
            String, '/executor/status', self.status_callback, status_qos
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10
        )

        self.create_timer(1.0, self.broadcast_sim_time_callback)

        # 데이터
        self.current_status = 'UNKNOWN'
        self.status_history = []
        self.map_data   = None
        self.robot_pose = {'x': 0, 'y': 0, 'theta': 0}

        self.get_logger().info('Patrol Control API Node initialized')

    # -------------------------------------------------------
    #  콜백
    # -------------------------------------------------------
    def status_callback(self, msg):
        self.current_status = msg.data
        self.status_history.append({
            'status': msg.data,
            'timestamp': self.get_clock().now().to_msg().sec
        })
        if len(self.status_history) > 100:
            self.status_history.pop(0)
        self.ws_manager.broadcast_status(self.current_status)

    def broadcast_sim_time_callback(self):
        
        now_ros = self.get_clock().now()

        total_seconds = now_ros.nanoseconds / 1e9
        m, _ = divmod(int(total_seconds), 60)
        h, m = divmod(m, 60)

        self.sim_time = f"{h % 24:02d}:{m:02d}"

        self.ws_manager.broadcast_sim_time(self.sim_time)


    def map_callback(self, msg):
        try:
            width  = msg.info.width
            height = msg.info.height

            map_array = np.array(msg.data).reshape((height, width))
            img_array = np.zeros((height, width), dtype=np.uint8)
            img_array[map_array == -1]  = 127
            img_array[map_array == 0]   = 255
            img_array[map_array == 100] = 0

            img = Image.fromarray(img_array, mode='L')
            img = img.transpose(Image.FLIP_TOP_BOTTOM)

            buffer = io.BytesIO()
            img.save(buffer, format='PNG')
            img_base64 = base64.b64encode(buffer.getvalue()).decode()

            self.map_data = {
                'image': img_base64,
                'width': width,
                'height': height,
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y
                }
            }
            self.ws_manager.broadcast_map(self.map_data)
            self.get_logger().info(f'Map: {width}x{height}')
        except Exception as e:
            self.get_logger().error(f'Map error: {e}')

    def pose_callback(self, msg):
        import math
        q     = msg.pose.pose.orientation
        theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.robot_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': theta
        }
        self.ws_manager.broadcast_pose(self.robot_pose)

    # -------------------------------------------------------
    # 명령 서비스 호출
    # -------------------------------------------------------
    async def call_service(self, command: str):
        if command == 'START_PATROL':
            client = self.cli_start
        elif command == 'RETURN_HOME':
            client = self.cli_home
        elif command == 'STOP':
            client = self.cli_stop
        else:
            return False, 'Unknown command'

        if not client.wait_for_service(timeout_sec=1.0):
            return False, 'Service not available'

        try:
            future   = client.call_async(Trigger.Request())
            response = await self._await_rclpy_future(future)
            return response.success, response.message
        except Exception as e:
            return False, str(e)

    # -------------------------------------------------------
    # rclpy Future 헬퍼
    # -------------------------------------------------------
    async def _await_rclpy_future(self, future, timeout: float = 5.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                raise TimeoutError('rclpy Future timeout')
            await asyncio.sleep(0.05)
        return future.result()

    # -------------------------------------------------------
    # 운영 설정
    # -------------------------------------------------------
    async def set_operation_config(self, config: dict):
        if not self.cli_set_params.wait_for_service(timeout_sec=1.0):
            return False, 'control_node set_parameters 서비스 없음'

        try:
            params = []
            for key, value in config.items():
                p       = Parameter()
                p.name  = key
                p.value = self._to_param_value(value)
                params.append(p)

            req            = SetParameters.Request()
            req.parameters = params

            future   = self.cli_set_params.call_async(req)
            response = await self._await_rclpy_future(future)

            all_ok = all(r.successful for r in response.results)
            return all_ok, '설정 적용 완료' if all_ok else '일부 파라미터 적용 실패'

        except Exception as e:
            return False, str(e)

    async def get_operation_config(self):
        if not self.cli_get_params.wait_for_service(timeout_sec=1.0):
            return None

        try:
            req       = GetParameters.Request()
            req.names = ['operation_start', 'operation_end', 'auto_patrol']

            future   = self.cli_get_params.call_async(req)
            response = await self._await_rclpy_future(future)

            result = {}
            for k, v in zip(req.names, response.values):
                if v.type == ParameterType.PARAMETER_STRING:
                    result[k] = v.string_value
                elif v.type == ParameterType.PARAMETER_BOOL:
                    result[k] = v.bool_value
                elif v.type == ParameterType.PARAMETER_INTEGER:
                    result[k] = v.integer_value
                elif v.type == ParameterType.PARAMETER_DOUBLE:
                    result[k] = v.double_value
            return result
        except Exception as e:
            self.get_logger().error(f'get_operation_config 오류: {e}')
            return None

    def _to_param_value(self, value):
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type       = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type          = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        elif isinstance(value, float):
            pv.type         = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        elif isinstance(value, str):
            pv.type         = ParameterType.PARAMETER_STRING
            pv.string_value = value
        return pv

    # -------------------------------------------------------
    # 상태 반환
    # -------------------------------------------------------
    def get_status(self):
        return {
            'status':  self.current_status,
            'history': self.status_history[-10:],
            'map':     self.map_data,
            'pose':    self.robot_pose
        }