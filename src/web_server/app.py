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
import yaml
import os


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
CONFIG_PATH = os.environ.get(
    'CONFIG_PATH',
    '/root/patrol_robot/src/patrol_control/config/operation_policy.yaml'
)


# -------------------------------------------------------
# Config 유틸
# -------------------------------------------------------
def load_config() -> dict:
    with open(CONFIG_PATH, 'r') as f:
        return yaml.safe_load(f)


def save_config(data: dict):
    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)


# -------------------------------------------------------
# 모델
# -------------------------------------------------------
class OperationConfig(BaseModel):
    """운영 설정 모델"""
    operation_start: Optional[str] = None
    operation_end:   Optional[str] = None
    auto_patrol:     Optional[bool] = None


# -------------------------------------------------------
# 앱 시작
# -------------------------------------------------------
@app.on_event("startup")
async def startup():
    """
    웹서버 시작 시 현재 YAML 설정 확인만 (노드가 이미 읽어서 적용함)
    """
    try:
        cfg = load_config().get('control_node', {}).get('ros__parameters', {})
        print(f"[WebServer] 현재 운영 설정: {cfg}")
    except Exception as e:
        print(f"[WebServer] Config 읽기 실패: {e}")


# -------------------------------------------------------
# API
# -------------------------------------------------------
@app.get("/api/config")
async def get_config():
    """현재 운영 설정 조회 (YAML 기준)"""
    config = load_config()
    result = config.get('control_node', {}).get('ros__parameters', {})
    raw_wps = config.get('waypoints', [])
    result['waypoints'] = [{'x': wp[0], 'y': wp[1]} for wp in raw_wps]
    raw_home = config.get('homepoint', [])
    result['homepoint'] = {'x': raw_home[0][0], 'y': raw_home[0][1]} if raw_home else None
    return result


@app.post("/api/config")
async def update_config(config: OperationConfig):
    """
    1. YAML 파일에 저장 (영구 반영 - 노드 재시작해도 유지)
    2. control_node 파라미터 즉시 업데이트
    """
    config_dict = config.model_dump(exclude_none=True)

    # 1. YAML 파일 업데이트
    try:
        current = load_config()
        current.setdefault('control_node', {}).setdefault('ros__parameters', {}).update(config_dict)
        save_config(current)
    except Exception as e:
        return {'success': False, 'message': f'파일 저장 실패: {e}'}

    # 2. control_node에 즉시 반영
    success, message = await patrol_node.set_operation_config(config_dict)

    return {'success': success, 'message': message}


@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    """메인 페이지"""
    return templates.TemplateResponse("index.html", {"request": request})


@app.get("/api/status")
async def get_status():
    """상태 및 설정 조회"""
    status = patrol_node.get_status()

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
        return {'success': False, 'message': f'Invalid command: {command}'}

    success, message = await patrol_node.call_service(command_upper)

    if success:
        config = await patrol_node.get_operation_config()
        if config:
            manager.broadcast_config(config)

    return {'success': success, 'message': message}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 연결"""
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)


# -------------------------------------------------------
# 실행
# -------------------------------------------------------
def run_fastapi():
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


def main():
    global patrol_node

    rclpy.init()
    patrol_node = PatrolControlNode(manager)

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
        print("\nShutting down...")
    finally:
        patrol_node.destroy_node()
        rclpy.shutdown()


def get_local_ip():
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


if __name__ == '__main__':
    main()