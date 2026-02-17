# ROS 2 Playground

## 기본 구성

### 1. Docker 기반 리눅스 설치 (Mac 용)

```bash
# 1. 호스트 OS
## 먼저 GUI 연동을 위해 Homebrew를 통해 XQuartz 설치
brew install --cask xquartz

## XQuartz 실행 (최초 1회 설정 필요)
open -a XQuartz

## 1. GUI 권한 부여
xhost +

# 2. Docker
## ROS2를 위한 Ubuntu 20.04 생성
## 패키지 설치 연습 목적으로 우분투에서 부터 시작
## DISPLAY는 컨테이너 내부 .bashrc에서 host.docker.internal:0 으로 자동 설정됨
docker run -it \
  --name ros2_foxy_m4 \
  --platform linux/arm64 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/.Xauthority:/root/.Xauthority:rw \
  -v $(pwd):/root/ros2_workspace \
  ubuntu:20.04 \
  /bin/bash
```

### 2. ROS 2 설치

```bash
# 지역 설정
apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 소스 설정
apt update && apt install curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. ROS 2 패키지 설치

```bash
apt update
apt install ros-foxy-desktop ros-foxy-rmw-fastrtps* ros-foxy-rmw-cyclonedds*
```

### 4. ROS 2 패키지 설치 확인

```bash
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker

# 결과
[INFO] [1770470934.027748962] [talker]: Publishing: 'Hello World: 1'
[INFO] [1770470935.026389296] [talker]: Publishing: 'Hello World: 2'
[INFO] [1770470936.027172713] [talker]: Publishing: 'Hello World: 3'
[INFO] [1770470937.026290797] [talker]: Publishing: 'Hello World: 4'
...

source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener

# Talker 실행 후 확인 가능
[INFO] [1770471039.680911219] [listener]: I heard: [Hello World: 1]
[INFO] [1770471040.671301803] [listener]: I heard: [Hello World: 2]
[INFO] [1770471041.669022137] [listener]: I heard: [Hello World: 3]
[INFO] [1770471042.673671137] [listener]: I heard: [Hello World: 4]
[INFO] [1770471043.669871096] [listener]: I heard: [Hello World: 5]
[INFO] [1770471044.672463055] [listener]: I heard: [Hello World: 6]
...
```

### 5. ROS 개발 툴 설치

```bash
apt update && apt install -y \
build-essential \
cmake \
libbullet-dev \
python3-colcon-common-extensions \
python3-flake8 \
python3-pip \
python3-pytest-cov \
python3-rosdep \
python3-setuptools \
python3-vcstool \
wget

python3 -m pip install -U \
argcomplete \
flake8-blind-except \
flake8-builtins \
flake8-class-newline \
flake8-comprehensions \
flake8-deprecated \
flake8-docstrings \
flake8-import-order \
flake8-quotes \
pytest-repeat \
pytest-rerunfailures \
pytest

apt install --no-install-recommends -y \
libasio-dev \
libtinyxml2-dev \
libcunit1-dev
```

### 6. ROS 2 빌드 테스트

```bash
source /opt/ros/foxy/setup.bash
mkdir -p ./src
colcon build --symlink-install
```

### 7. setup.bash 자동화

```bash
# vim 설치 필수
vim ~/.bashrc

# 추가
source /opt/ros/foxy/setup.bash
source ~/ros2_workspace/install/local_setup.bash

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/ros2_workspace

export ROS_DOMAIN_ID=7
export ROS_NAMESPACE=robot1

export DISPLAY=host.docker.internal:0

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name})() at {file_name}:{line_number}'
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=0

alias cw='cd ~/ros2_workspace'
alias cs='cd ~/ros2_workspace/src'
alias ccd='colcon_cd'

alias cb='cd ~/ros2_workspace && colcon build --symlink-install'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias cbu='colcon build --symlink-install --packages-up-to'
alias ct='colcon test'
alias ctp='colcon test --packages-select'
alias ctr='colcon test-result'

alias rt='ros2 topic list'
alias re='ros2 topic echo'
alias rn='ros2 node list'

alias killgazebo='killall -9 gazebo & killall -9 gzserver & killall -9 gzclient'

alias af='ament_flake8'
alias ac='ament_cpplint'

alias testpub='ros2 run demo_nodes_cpp talker'
alias testsub='ros2 run demo_nodes_cpp listener'
alias testpubimg='ros2 run image_tools cam2image'
alias testsubimg='ros2 run image_tools showimage'
```

### 8. VSCode 개발환경 설정

1. User settings
    ```json
    {
      "ROS2.distro": "foxy",
      "colcon.provideTasks": true,
      "files.associations": {
        "*.repos": "yaml",
        "*.world": "xml",
        "*.xacro": "xml"
      }
    }
    ```
2. ~/.config.Code/User/settings.json
    ```json
    {
      "cmake.configureOnOpen": false,
      "editor.minimap.enabled": false,
      "editor.mouseWheelZoom": true,
      "editor.renderControlCharacters": true,
      "editor.rulers": [120],
      "editor.tabSize": 2,
      "files.associations": {
        "*.repos": "yaml",
        "*.world": "xml",
        "*.xacro": "xml"
      },
      "files.insertFinalNewline": true,
      "files.trimTrailingWhitespace": true,
      "terminal.integrated.scrollback": 1000000,
      "workbench.iconTheme": "vscode-icons",
      "workbench.editor.pinnedTabSizing": "compact",
      "ros.distro": "foxy",
      "colcon.provideTasks": true
    }
    ```
3. c_cpp_properties.json
    ```json
    {
      "configurations": [
        {
          "name": "Linux",
          "includePath": [
            "${default}",
            "${workspaceFolder}/**",
            "/opt/ros/foxy/include/**"
          ],
          "defines": [],
          "compilerPath": "/usr/bin/g++",
          "cStandard": "c99",
          "cppStandard": "c++14",
          "intelliSenseMode": "linux-gcc-x64"
        }
      ],
      "version": 4
    }
    ```
4. tasks.json
    ```json
    {
      "version": "2.0.0",
      "tasks": [
        {
          "label": "colcon: build",
          "type": "shell",
          "command": "colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Debug'",
          "problemMatcher": [],
          "group": {
            "kind": "build",
            "isDefault": true
          }
        },
        {
          "label": "colcon: test",
          "type": "shell",
          "command": "colcon test && colcon test-result"
        },
        {
          "label": "colcon: clean",
          "type": "shell",
          "command": "rm -rf build install log"
        }
      ]
    }
    ```
5. Launch 설정`launch.json`
    ```json
    {
      "version": "0.2.0",
      "configurations": [
        {
          "name": "Debug-rclpy(debugpy)",
          "type": "debugpy",
          "request": "launch",
          "program": "${file}",
          "console": "integratedTerminal"
        },
        {
          "name": "Debug-rclcpp(gdb)",
          "type": "cppdbg",
          "request": "launch",
          "program": "${workspaceRoot}/install/${input:package}/lib/${input:package}/${input:node}",
          "args": [],
          "preLaunchTask": "colcon: build",
          "stopAtEntry": true,
          "cwd": "${workspaceFolder}",
          "externalConsole": false,
          "MIMode": "gdb",
          "setupCommands": [
            {
              "description": "Enable pretty-printing for gdb",
              "text": "-enable-pretty-printing",
              "ignoreFailures": true
            }
          ]
        },
      ],
      "inputs": [
        {
          "id": "package",
          "type": "promptString",
          "description": "package name",
          "default": "topic_service_action_rclcpp_example"
        },
        {
          "id": "node",
          "type": "promptString",
          "description": "node name",
          "default": "argument"
        }
      ]
    }
    ```

### 9. QtCreator 설치
```bash
apt install qtcreator
```

### 10. Turtlesim
```bash
# 설치
sudo apt update && apt install ros-foxy-turtlesim

# 실행
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

# 덤프파일 기반 ros2 실행
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
```
