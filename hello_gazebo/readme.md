# Hello Gazebo 워크스페이스

ROS2 Jazzy와 Gazebo를 활용한 교육용 자율주행 로봇 시뮬레이션 플랫폼의 메인 워크스페이스입니다.

## 개요

이 워크스페이스는 라이다 센서가 장착된 스마트 차량의 SLAM, 자율주행, 키보드 제어 기능을 제공합니다.

## 패키지 구성

- **`hello_gazebo_bringup`** - 통합 실행 및 시뮬레이션 환경
- **`hello_gazebo_description`** - 로봇 URDF 모델 정의  
- **`hello_gazebo_slam`** - SLAM 매핑 및 시각화
- **`hello_gazebo_navigation`** - Navigation2 자율주행

## 빠른 시작

### 1. 환경 설정
```bash
cd ~/dev_ws/Hello_Gazebo/hello_gazebo
source install/setup.bash
```

### 2. 기본 시뮬레이션 실행
```bash
# Gazebo 시뮬레이션만 실행 (RViz 분리됨)
ros2 launch hello_gazebo_bringup launch_sim.launch.xml

# 다른 월드 환경 사용
ros2 launch hello_gazebo_bringup launch_sim.launch.xml world_name:=simple_world.world

# 로봇 초기 위치 변경
ros2 launch hello_gazebo_bringup launch_sim.launch.xml x:=10.0 y:=-10.0 z:=0.3
```

### 3. RViz 시각화 (상황별 선택 실행)
⚠️ **주의**: 상황에 맞게 **하나씩만** 실행하세요!

```bash
# SLAM 매핑 시: SLAM용 RViz만 실행
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash  
ros2 launch hello_gazebo_slam rviz_slam.launch.xml

# 또는

# 자율주행 시: Navigation용 RViz만 실행
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_navigation rviz_navigation.launch.xml
```

### 4. SLAM 매핑
```bash
# 1단계: 시뮬레이션 실행
ros2 launch hello_gazebo_bringup launch_sim.launch.xml

# 2단계: SLAM용 RViz 실행 (새 터미널)
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_slam rviz_slam.launch.xml

# 3단계: SLAM 시작 (새 터미널)
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_slam slam_mapping.launch.xml

# 4단계: 키보드 제어 (새 터미널)
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 5단계: 맵 저장
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 5. 자율주행 내비게이션
```bash
# 1단계: 시뮬레이션 실행
ros2 launch hello_gazebo_bringup launch_sim.launch.xml

# 2단계: Navigation용 RViz 실행 (새 터미널)
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_navigation rviz_navigation.launch.xml

# 3단계: 위치추정 시작 (새 터미널)
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_navigation localization.launch.xml \
    map:=$(find-pkg-share hello_gazebo_navigation)/map/my_factory_map.yaml

# 4단계: 내비게이션 시작 (새 터미널)
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_navigation navigation.launch.xml
```

## 고급 사용법 - 개별 런치 파일 활용

### 📍 위치추정 시스템 (localization.launch.xml)

저장된 맵을 사용하여 로봇의 정확한 위치를 추정합니다.

```bash
# 1단계: 시뮬레이션 실행 (첫 번째 터미널)
ros2 launch hello_gazebo_bringup launch_sim.launch.xml

# 2단계: 위치추정 시작 (두 번째 터미널)
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_navigation localization.launch.xml \
    map:=$(find-pkg-share hello_gazebo_navigation)/map/my_factory_map.yaml

# 다른 맵 파일 사용하기
ros2 launch hello_gazebo_navigation localization.launch.xml \
    map:=/path/to/your/custom_map.yaml
```

### 🚀 내비게이션 시스템 (navigation.launch.xml)

경로 계획 및 자율주행 기능을 제공합니다.

```bash
# 3단계: 내비게이션 스택 시작 (세 번째 터미널)
cd ~/dev_ws/Hello_Gazebo/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_navigation navigation.launch.xml

# 커스텀 파라미터 사용
ros2 launch hello_gazebo_navigation navigation.launch.xml \
    params_file:=/path/to/custom_nav2_params.yaml
```

### 🎯 목표점 설정 방법

#### RViz에서 목표점 설정
1. RViz 상단의 **"2D Goal Pose"** 버튼 클릭
2. 맵에서 목표 위치 클릭 후 드래그하여 방향 설정
3. 로봇이 자동으로 해당 위치로 이동

#### 명령어로 목표점 설정
```bash
# 특정 좌표로 자율주행 명령
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"},
  pose: {
    position: {x: 10.0, y: -5.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

### 📊 내비게이션 모니터링

RViz에서 확인할 수 있는 요소들:
- 🗺️ **맵**: 저장된 환경 맵
- 🔴 **Global Costmap**: 전역 비용 맵 (빨간색 = 장애물)
- 🔵 **Local Costmap**: 지역 비용 맵 (로봇 주변)
- 🟢 **Global Path**: 전역 경로 계획 (녹색 선)
- 🔴 **Local Path**: 지역 경로 계획 (빨간색 선)
- 🟡 **Particle Cloud**: AMCL 위치추정 파티클들

### ⚠️ 실행 순서 주의사항

1. **반드시 순서대로 실행**: 시뮬레이션 → 위치추정 → 내비게이션
2. **맵 파일 필수**: localization 실행 전 맵 파일 경로 확인
3. **각각 별도 터미널**: 각 런치 파일은 다른 터미널에서 실행

## 저장된 맵 파일

- `my_map.pgm` / `my_map.yaml` - factory_L1 환경의 SLAM 매핑 결과

## 주요 제어 키 (키보드 제어 모드)

- **이동**: `i`(전진), `k`(정지), `j`(좌회전), `l`(우회전)
- **속도**: `q`/`z`(선속도 증가/감소), `w`/`x`(각속도 증가/감소)
- **종료**: `Ctrl+C`

## 문제 해결

### 빌드 오류 시
```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Gazebo 실행 오류 시
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch hello_gazebo_bringup launch_sim.launch.xml
```

## 추가 정보

전체 프로젝트 문서 및 상세 사용법은 [메인 README](../README.md)를 참조하세요.

---

**Happy Robot Programming!** 🤖
