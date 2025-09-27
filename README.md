# Hello Gazebo

ROS 2 + Gazebo로 SLAM과 자율주행 실습한 개인 프로젝트

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

## SLAM 실습

```bash
# SLAM 시작
ros2 launch hello_gazebo_slam slam_mapping.launch.xml
ros2 launch hello_gazebo_slam rviz_slam.launch.xml

# 키보드로 조종하며 맵 생성
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 자율주행 실습

```bash
# 내비게이션 시작  
ros2 launch hello_gazebo_navigation localization.launch.xml
ros2 launch hello_gazebo_navigation navigation.launch.xml
ros2 launch hello_gazebo_navigation rviz_navigation.launch.xml

# RViz에서 목표점 설정하여 자율주행
```