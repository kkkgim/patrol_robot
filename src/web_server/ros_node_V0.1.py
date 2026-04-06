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

import base64
import numpy as np
from PIL import Image
import io
import asyncio


class PatrolControlNode(Node):

    def __init__(self, websocket_manager):
        super().__init__('patrol_control_api')
        
        self.ws_manager = websocket_manager
        
        # 서비스 클라이언트
        self.cli_start = self.create_client(Trigger, '/control/start_patrol')
        self.cli_home = self.create_client(Trigger, '/control/return_home')
        self.cli_stop = self.create_client(Trigger, '/control/stop')
        
        # 파라미터 클라이언트
        self.cli_get_params = self.create_client(
            GetParameters,
            '/control_node/get_parameters'
        )
        self.cli_set_params = self.create_client(
            SetParameters,
            '/control_node/set_parameters'
        )
        
        # 구독자
        self.status_sub = self.create_subscription(
            String,
            '/executor/status',
            self.status_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # 데이터
        self.current_status = "UNKNOWN"
        self.status_history = []
        self.map_data = None
        self.robot_pose = {'x': 0, 'y': 0, 'theta': 0}
        
        self.get_logger().info('Patrol Control API Node initialized')
    
    def status_callback(self, msg):
        """상태 업데이트"""
        self.current_status = msg.data
        self.status_history.append({
            'status': msg.data,
            'timestamp': self.get_clock().now().to_msg().sec
        })
        if len(self.status_history) > 100:
            self.status_history.pop(0)
        
        self.ws_manager.broadcast_status(self.current_status)
    
    def map_callback(self, msg):
        """지도 수신 및 변환"""
        try:
            width = msg.info.width
            height = msg.info.height
            
            map_array = np.array(msg.data).reshape((height, width))
            img_array = np.zeros((height, width), dtype=np.uint8)
            img_array[map_array == -1] = 127
            img_array[map_array == 0] = 255
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
        """로봇 위치 업데이트"""
        import math
        q = msg.pose.pose.orientation
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
    
    async def call_service(self, command: str):
        """명령 서비스 호출"""
        if command == 'START_PATROL':
            client = self.cli_start
        elif command == 'RETURN_HOME':
            client = self.cli_home
        elif command == 'STOP':
            client = self.cli_stop
        else:
            return False, "Unknown command"

        if not client.wait_for_service(timeout_sec=1.0):
            return False, "Service not available"

        try:
            future = client.call_async(Trigger.Request())
            response = await future
            return response.success, response.message
        except Exception as e:
            return False, str(e)
    

    async def set_operation_config(self, config: dict):
        try:
            # dict → ROS2 Parameter 리스트로 변환
            params = []
            for key, value in config.items():
                p = Parameter()
                p.name = key
                p.value = self._to_param_value(value)
                params.append(p)

            # control_node에 파라미터 세팅
            client = self.node.create_client(SetParameters, '/control_node/set_parameters')
            req = SetParameters.Request()
            req.parameters = params
            future = client.call_async(req)
            await asyncio.wrap_future(future)

            return True, "설정 적용 완료"
        except Exception as e:
            return False, str(e)

    def _to_param_value(self, value):
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        elif isinstance(value, str):
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = value
        return pv
    
    def get_status(self):
        """현재 상태 반환"""
        return {
            'status': self.current_status,
            'history': self.status_history[-10:],
            'map': self.map_data,
            'pose': self.robot_pose
        }