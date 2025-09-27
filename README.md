![banner](https://github.com/jinhyuk2me/Gazebo_with_SLAM_and_Nav2/blob/main/assets/banner.png?raw=true)

# Gazebo_with_SLAM_and_Nav2

Gazebo 시뮬레이션 내에서의 SLAM & Nav2

## 구조

```
hello_gazebo/
├── hello_gazebo_bringup/     # 시뮬레이션 실행
├── hello_gazebo_description/ # 로봇 모델
├── hello_gazebo_slam/        # SLAM 매핑
└── hello_gazebo_navigation/  # 자율주행
```

## 빌드 & 실행

```bash
cd hello_gazebo
colcon build --symlink-install
source install/setup.bash

# 시뮬레이션 시작
ros2 launch hello_gazebo_bringup launch_sim.launch.xml
```

## SLAM

![slam](https://github.com/jinhyuk2me/Gazebo_with_SLAM_and_Nav2/blob/main/assets/slam.gif?raw=true)

```bash
# SLAM 시작
ros2 launch hello_gazebo_slam slam_mapping.launch.xml
ros2 launch hello_gazebo_slam rviz_slam.launch.xml

# 키보드로 조종하며 맵 생성
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 자율주행

```bash
# 위치추정 시작 (맵 파일 지정 필수)
ros2 launch hello_gazebo_navigation localization.launch.xml \
    map:=$(find-pkg-share hello_gazebo_navigation)/map/my_factory_map.yaml

# 내비게이션 시작
ros2 launch hello_gazebo_navigation navigation.launch.xml
ros2 launch hello_gazebo_navigation rviz_navigation.launch.xml

# RViz에서 목표점 설정하여 자율주행
```
