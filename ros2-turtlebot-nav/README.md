# 🤖 ROS2 Turtlebot3 Navigation & Mission Control
**Turtlebot3 Waffle Pi를 활용한 실내 자율주행 및 세탁물 집기/이송 패키지입니다.**

## 🏗️ Workspaces & Path
프로젝트의 각 기능이 위치한 경로입니다.

### Remote PC (Laptop)
* `~/ros2_ws/src/gripper_controller/`: 그리퍼 원격 제어 노드
* `~/turtlebot3_ws/src/turtlebot3_teleop/`: 키보드 제어 스크립트

### Robot (Turtlebot2 / Raspberry Pi)
* `~/ros2_ws/src/gripper_bridge/`: STM32 통신 브릿지 노드
* `~/turtlebot3_ws/src/turtlebot3_bringup/`: 하드웨어 구동 런치 파일

---

## 💻 가상 환경 (Gazebo) 실행 순서
시뮬레이션 환경에서 Nav2와 비전 미션을 테스트하는 단계입니다.

1. **사전 준비 (환경 변수 설정)**
   ```bash
   export TURTLEBOT3_MODEL=burger
   source /opt/ros/humble/setup.bash
   source ~/turtlebot3_ws/install/setup.bash
   ```

2. **Gazebo 월드 및 모델 로드**
   * `ros2 launch turtlebot3_gazebo empty_world.launch.py` 실행
   * 이후 [Insert] 메뉴에서 `final_world`와 `conveyor` 모델을 불러옵니다.

3. **Navigation2 실행**
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=~/final_map_1.yaml
   ```
   * **주의**: 맵 작성 시와 동일한 월드를 사용해야 하며, `use_sim_time:=True` 옵션이 필수입니다.

4. **위치 지정 및 카메라 노드 실행**
   * **위치 초기화**: Rviz2의 `2D Pose Estimate` 버튼을 사용해 위치를 찍고, `teleop_keyboard`로 정밀 조정합니다.
   * **카메라 구동**:
     ```bash
     ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[160,160] -p framerate:=15.0 -p video_device:="/dev/video1"
     ```

5. **통합 미션 스크립트 실행**
   ```bash
   python3 final_patrol_grip.py
   ```

---

## 🐢 실제 Turtlebot3 Waffle Pi 실행 순서
실제 하드웨어를 구동하여 실내 자율주행 및 물건 집기 미션을 수행합니다.

### Step 1: 로봇 Bringup (로봇 SSH 접속)
```bash
# 로봇 접속
ssh [로봇ID]@[로봇IP]

# 하드웨어 구동
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py usb_port:=/dev/ttyACM1
```
* **성공 신호**: "띠리링" 소리와 함께 LiDAR 센서 회전을 확인합니다.

### Step 2: 카메라 및 제어 브릿지 실행 (로봇 쪽 터미널)
* **파이 카메라 노드**:
  ```bash
  ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[160,160] -p framerate:=2.0 --video_device:="/dev/video0"
  ```
* **그리퍼 브릿지 노드**:
  ```bash
  ros2 run gripper_bridge gripper_bridge
  ```

### Step 3: 원격 PC 제어 및 주행 (Laptop 터미널)
1. **Nav2 실행**:
   `ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/final_map_1.yaml`
2. **그리퍼 제어 및 미션 스크립트 실행**:
   `ros2 run gripper_controller gripper_controller`
   `python3 final_patrol_grip.py`

---

## 🛠️ Troubleshooting (자주 발생하는 문제)

1. **시간 동기화 오류 (TF Transform 문제)**
   * PC와 로봇의 시스템 시간이 맞지 않으면 자율주행 경로 생성 시 에러가 발생합니다.
   * **해결**: `sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"` (양쪽 모두 실행 권장)

2. **DDS 통신 문제 (노드 인식 안 됨)**
   * PC와 로봇의 `ROS_DOMAIN_ID`가 동일해야 합니다.
   * `echo $ROS_DOMAIN_ID` 명령어로 값을 확인하세요.

3. **장애물이 없는데 이동 불가 판정**
   * 현재 실제 환경이 작성된 맵(`map.yaml`)과 많이 다를 경우 발생합니다. 장애물을 치우거나 맵을 새로 생성해야 합니다.

---

## 📈 추가 개발: 거리 측정 노드 (Distance Checker)
2차원 좌표 기반의 거리 측정을 위한 추가 패키지 구성입니다.

* **패키지 생성**:
  `ros2 pkg create --build-type ament_python turtlebot_distance_checker --dependencies rclpy sensor_msgs cv_bridge std_msgs`
* **소스 위치**: `~/ros2_ws/src/turtlebot_distance_checker/turtlebot_distance_checker/distance_node.py`
* **빌드 및 적용**: `colcon build && source ~/.bashrc`