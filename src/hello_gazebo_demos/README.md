# Hello Gazebo 데모 패키지 🚗

VicPinky 방식을 참고한 초보자용 Gazebo + ROS 2 통합 데모입니다.

## 📋 필요한 패키지들

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-xacro
```

## 🎯 데모 실행 방법

### 1. 전체 시뮬레이션 실행 (추천)
```bash
# 빌드
cd ~/dev_ws/Hello_Gazebo
colcon build --packages-select hello_gazebo_demos

# 전체 시뮬레이션 실행
source install/setup.bash
ros2 launch hello_gazebo_demos launch_sim.launch.xml
```

### 2. 단계별 실행

**2-1. Gazebo만 실행**
```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$(ros2 pkg prefix hello_gazebo_demos)/share/hello_gazebo_demos/worlds/simple_world.world"
```

**2-2. 로봇 스폰 (새 터미널)**
```bash
source install/setup.bash
ros2 launch hello_gazebo_demos spawn_vehicle.launch.xml
```

### 3. 키보드 조종

**새 터미널에서:**
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

## 🎮 키보드 조작법

```
   u    i    o
   j    k    l
   m    ,    .

i: 전진 ⬆️
,: 후진 ⬇️
j: 좌회전 ↺
l: 우회전 ↻
k: 정지 ⏹️
q: 종료 ❌
```

## 📊 확인할 수 있는 토픽들

```bash
# 모든 토픽 확인
ros2 topic list

# 차량 속도 명령 확인
ros2 topic echo /cmd_vel

# 오도메트리 데이터 확인
ros2 topic echo /odom

# 카메라 영상 확인
ros2 topic echo /camera/image_raw

# TF 변환 확인
ros2 topic echo /tf
```

## 🔧 구조 설명

### VicPinky 방식 특징
1. **XML 런치 파일**: Python 대신 XML 사용
2. **URDF + Xacro**: 로봇 모델을 URDF로 정의
3. **YAML 브리지 설정**: 브리지 설정을 YAML 파일로 분리
4. **모듈화**: 시뮬레이션과 로봇 스폰을 분리

### 파일 구조
```
hello_gazebo_demos/
├── launch/
│   ├── launch_sim.launch.xml      # 전체 시뮬레이션 실행
│   └── spawn_vehicle.launch.xml   # 로봇만 스폰
├── worlds/
│   └── simple_world.world         # 간단한 시뮬레이션 월드
├── urdf/
│   └── smart_vehicle.urdf.xacro   # 차량 모델 정의
├── params/
│   └── vehicle_bridge.yaml        # ROS-Gazebo 브리지 설정
└── rviz/
    └── smart_vehicle.rviz          # RViz 설정
```

## 🛠️ 문제 해결

### Gazebo가 실행되지 않는 경우
```bash
# Gazebo 환경 변수 확인
echo $GZ_SIM_RESOURCE_PATH

# 패키지 재빌드
colcon build --packages-select hello_gazebo_demos --cmake-clean-cache
```

### 로봇이 스폰되지 않는 경우
```bash
# robot_description 토픽 확인
ros2 topic echo /robot_description --once

# URDF 파일 검증
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix hello_gazebo_demos)/share/hello_gazebo_demos/urdf/smart_vehicle.urdf.xacro)"
```

## 🎓 학습 포인트

1. **XML 런치 파일**: 더 구조화된 런치 시스템
2. **URDF/Xacro**: 표준 로봇 모델링 방법
3. **YAML 설정**: 브리지 설정의 모듈화
4. **VicPinky 패턴**: 실제 로봇 개발에서 사용되는 패턴

즐거운 로봇 시뮬레이션 되세요! 🤖✨