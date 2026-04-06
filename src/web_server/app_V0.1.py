#!/usr/bin/env python3
"""
Patrol Robot Web Control - FastAPI 서버
"""
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel
import threading
import uvicorn
from pathlib import Path
from typing import Optional

from ros_node import PatrolControlNode
from websocket_manager import ConnectionManager

import rclpy
import yaml, os


# FastAPI 앱
app = FastAPI(title="Patrol Robot Control")

# 템플릿 설정
BASE_DIR = Path(__file__).resolve().parent
templates = Jinja2Templates(directory=str(BASE_DIR / "templates"))

# WebSocket 매니저
manager = ConnectionManager()

# 전역 노드
patrol_node = None

# 운영설정 파일 경로
CONFIG_PATH = os.environ.get('CONFIG_PATH', '/root/patrol_robot/src/patrol_control/config/operation_policy.yaml')

def load_config() -> dict:
    with open(CONFIG_PATH, 'r') as f:
        return yaml.safe_load(f)


def save_config(data: dict):
    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
     
   
class OperationConfig(BaseModel):
    """운영 설정 모델"""
    operation_start: Optional[str] = None
    operation_end: Optional[str] = None
    auto_patrol: Optional[bool] = None


# --- 앱 시작 시 config 읽어서 ROS2 노드에 적용 ---
@app.on_event("startup")
async def startup():

    print(f"[Startup] Config 로드 완료: {cfg}")

# --- 웹에서 수정 ---
@app.post("/api/config")
async def update_config(config: OperationConfig):
    config_dict = config.model_dump(exclude_none=True)

    # 1. 기존 파일 읽기
    current = load_config()
    current['operation'].update(config_dict)

    # 2. 파일에 저장
    save_config(current)

    # 3. ROS2 노드에 즉시 반영
    success, message = await patrol_node.set_operation_config(config_dict)

    return {'success': success, 'message': message}

# --- 현재 설정 조회 ---
@app.get("/api/config")
async def get_config():
    return load_config().get('operation', {})

@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    """메인 페이지"""
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/api/status")
async def get_status():
    """상태 및 설정 조회"""
    status = patrol_node.get_status()
    
    # 운영 설정 추가
    config = await patrol_node.get_operation_config()
    if config:
        status['config'] = config
    
    return status


@app.post("/api/command/{command}")
async def send_command(command: str):
    """명령 전송"""
    valid_commands = ['START_PATROL', 'RETURN_HOME', 'STOP']
    
    command_upper = command.upper()
    if command_upper not in valid_commands:
        return {
            'success': False,
            'message': f'Invalid command: {command}'
        }
    
    success, message = await patrol_node.call_service(command_upper)
    
    # 설정 변경 후 브로드캐스트
    if success:
        config = await patrol_node.get_operation_config()
        if config:
            manager.broadcast_config(config)
    
    return {
        'success': success,
        'message': message
    }



@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 연결"""
    await manager.connect(websocket)
    
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)


def run_fastapi():
    """FastAPI 서버 실행"""
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


def main():
    """메인 함수"""
    global patrol_node
    
    rclpy.init()
    patrol_node = PatrolControlNode(manager)
    
    # ROS2 spin을 별도 스레드에서
    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(patrol_node,),
        daemon=True
    )
    ros_thread.start()
    
    print("=" * 60)
    print("Patrol Robot Web Control")
    print("=" * 60)
    print(f"Local:   http://localhost:8000")
    print(f"Network: http://{get_local_ip()}:8000")
    print("=" * 60)
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        run_fastapi()
    except KeyboardInterrupt:
        print("\n Shutting down...")
    finally:
        patrol_node.destroy_node()
        rclpy.shutdown()


def get_local_ip():
    """로컬 IP 주소"""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"


if __name__ == '__main__':
    main()