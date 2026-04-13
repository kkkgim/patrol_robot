# ROS2 기반 자율 순찰 로봇 시스템

ROS2 Humble + Nav2 + Gazebo 시뮬레이션 환경에서 TurtleBot3를 활용한 자율 순찰 로봇 시스템입니다.  
웹 인터페이스를 통해 로봇을 원격 제어하고, waypoint 기반 자율 순찰 및 홈 복귀 기능을 구현했습니다.

---

## 프로젝트 개요

| 항목 | 내용 |
|------|------|
| 플랫폼 | TurtleBot3 (Gazebo 시뮬레이션) |
| ROS 버전 | ROS2 Humble |
| 네비게이션 | Nav2 + AMCL |
| 환경 | Ubuntu 22.04, Python 3 |

---

## 시스템 구조

```
[Web Server (Flask)]
        ↕ HTTP
[web_node]  ←→  [control_node]  →  [executor_node]
                                          ↕
                                    [Nav2 Stack]
                                          ↕
                                   [TurtleBot3 (Gazebo)]
```

### 노드 설명

- **web_node**: 웹 서버와 ROS2 간 인터페이스. 웹에서 받은 명령을 ROS2 토픽으로 변환하여 control_node에 전달
- **control_node**: 순찰 상태 관리 및 명령 처리. waypoint/homepoint 좌표를 executor_node에 전달
- **executor_node**: Nav2 Action Client로 목표 좌표를 전송하고 네비게이션 실행

---

## 주요 기능

- **Waypoint 자율 순찰**: 지정된 waypoint를 향해 자율 주행
- **Home 복귀**: 순찰 완료 후 출발 지점(homepoint)으로 자동 복귀
- **웹 원격 제어**: Flask 기반 웹 인터페이스로 순찰 시작/정지 제어
- **실시간 상태 전달**: 노드 간 파라미터 기반 실시간 데이터 동기화

---

## 🚀 실행 방법

### 1. Gazebo 시뮬레이션 환경 실행
```bash
ros2 launch patrol_control small_warehouse_tb3.launch.py
```

### 2. Nav2 네비게이션 스택 실행
```bash
ros2 launch patrol_control bringup_launch.py slam:=False use_sim_time:=True
```

### 3. 제어 노드 실행
```bash
ros2 run patrol_control control_node
ros2 run patrol_control executor_node
```

### 4. 웹 서버 실행
```bash
cd /root/patrol_robot/src/web_server && python3 app.py
```

---

## 트러블슈팅

### 1. 파라미터 콜백 내 데이터 동기화 이슈

**문제**  
`add_on_set_parameters_callback` 내에서 `get_parameter()`를 호출하면, 파라미터가 아직 커밋(Commit)되기 전 상태라 이전 값을 반환하는 문제 발생.  
web_node → control_node로 데이터를 넘길 때 실시간 반영이 되지 않아 제어 명령이 지연됨.

**해결**  
콜백으로 전달된 `Parameter` 객체 리스트를 직접 파싱하여 변수에 즉시 반영.  
`get_parameter()` 대신 콜백 인자를 직접 순회하여 값을 추출하는 방식으로 수정.

```python
def parameter_callback(self, params):
    for param in params:
        if param.name == 'target_waypoint':
            self.target_waypoint = param.value  # 직접 파싱
    return SetParametersResult(successful=True)
```

---

### 2. Nav2 실행 시 시간 동기화 문제 (use_sim_time)

**문제**  
Gazebo 시뮬레이터의 가상 시간과 실제 시스템 시간이 불일치하여 노드 간 TF(좌표 변환) 통신 오류 발생.

**해결**  
Nav2, AMCL, Robot State Publisher 등 모든 관련 노드의 `use_sim_time` 파라미터를 `True`로 통일.

```bash
ros2 launch patrol_control bringup_launch.py slam:=False use_sim_time:=True
```

---

### 3. AMCL 초기 위치 설정 문제

**문제**  
AMCL 기반 Localization은 로봇의 초기 위치 정보가 없으면 위치 추정을 시작하지 못함.

**해결**  
두 가지 방법으로 해결 가능.
- RViz의 `2D Pose Estimate` 툴로 초기 위치 수동 설정
- YAML 파일의 `amcl` 섹션에 시작 좌표를 직접 입력

---

## 사용 기술

| 분류 | 기술 |
|------|------|
| 로봇 미들웨어 | ROS2 Humble |
| 네비게이션 | Nav2, AMCL |
| 시뮬레이션 | Gazebo, TurtleBot3 |
| 언어 | Python3 |
| 웹 서버 | Flask |
| 환경 | Ubuntu 22.04, Linux |
| 형상관리 | Git |

---

## 관련 학습 기록

- [Velog - ROS2 및 C++ 학습 정리](https://velog.io/@kkkgim)