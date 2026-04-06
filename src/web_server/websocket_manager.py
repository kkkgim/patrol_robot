#!/usr/bin/env python3
"""
WebSocket 연결 관리자
"""
from fastapi import WebSocket
from typing import List
import json
import asyncio

class ConnectionManager:
    """WebSocket 연결 관리"""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        
    async def connect(self, websocket: WebSocket):
        """클라이언트 연결"""
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"Client connected. Total: {len(self.active_connections)}")
        
    def disconnect(self, websocket: WebSocket):
        """클라이언트 연결 해제"""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            print(f"Client disconnected. Total: {len(self.active_connections)}")
        
    async def broadcast(self, message: str):
        """모든 클라이언트에게 메시지 브로드캐스트"""
        disconnected = []
        
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                print(f"Error broadcasting: {e}")
                disconnected.append(connection)
        
        for conn in disconnected:
            self.disconnect(conn)
    
    def broadcast_map(self, map_data: dict):
        """지도 브로드캐스트"""
        message = json.dumps({
            'type': 'map_update',
            'data': map_data
        })
        self._safe_broadcast(message)
    
    
    def broadcast_status(self, status: str):
        """상태 브로드캐스트"""
        message = json.dumps({
            'type': 'status_update',
            'status': status
        })
        self._safe_broadcast(message)
    
    def broadcast_pose(self, pose: dict):
        """로봇 위치 브로드캐스트"""
        message = json.dumps({
            'type': 'pose_update',
            'pose': pose
        })
        self._safe_broadcast(message)

    def broadcast_sim_time(self, time_str: str):
        message = json.dumps({
            'type': 'sim_time',
            'time': time_str
        })
        self._safe_broadcast(message) 

    def broadcast_config(self, config: dict):
        """설정 브로드캐스트"""
        message = json.dumps({
            'type': 'config_update',
            'config': config
        })
        self._safe_broadcast(message)
    
    def _safe_broadcast(self, message: str):
        """비동기 브로드캐스트 (동기 호출용)"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        
        if loop.is_running():
            asyncio.create_task(self.broadcast(message))
        else:
            loop.run_until_complete(self.broadcast(message))