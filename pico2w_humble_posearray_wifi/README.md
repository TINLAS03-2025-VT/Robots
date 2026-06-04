# Pico 2 W Humble micro-ROS Wi-Fi PoseArray Live Movement

This folder contains the Pico 2 W firmware for receiving robot position updates over micro-ROS Wi-Fi UDP and driving the robot using the movement logic from the older live-movement branch.

The important design rule for this branch is:

~~~text
Keep the working Wi-Fi / UDP / micro-ROS transport from feature/pico2w-humble-posearray-wifi.
Import only the robot movement logic from feature/live-movement.
Do not use the old Wi-Fi, UDP, ROS, or transport code from feature/live-movement.
~~~

## What this firmware does

The Pico 2 W:

1. Connects to Wi-Fi.
2. Connects to a micro-ROS Agent over UDP.
3. Publishes a counter heartbeat on `/pico_counter`.
4. Subscribes to robot positions on `/robots/pos`.
5. Uses `poses[robot_num]` from the incoming `PoseArray` as this robot's current position.
6. Drives toward a fixed target pose at `(0, 0, 0)` using the imported movement code.
7. Prints movement debug messages such as turning left, turning right, moving forward, or stopping.

Published topic:

~~~text
/pico_counter std_msgs/msg/UInt64
~~~

Subscribed topic:

~~~text
/robots/pos geometry_msgs/msg/PoseArray
~~~

## Source and credits

This branch combines code from:

- `feature/pico2w-humble-posearray-wifi`  
  Used as the working Pico 2 W Wi-Fi UDP micro-ROS base.

- `feature/live-movement` / uploaded `Robots-feature-live-movement` repo  
  Used only for robot movement functionality, especially:
  - `interface/movement/movement.c`
  - `interface/movement/movement.h`
  - `settings.h`

- `RIPLaboratoryUH/pico2w-microros-wifi-guide`  
  Original Pico 2 W Wi-Fi UDP micro-ROS example that the working Wi-Fi branch was based on.

- `micro-ROS/micro_ros_raspberrypi_pico_sdk`  
  Used for the ROS 2 Humble `libmicroros` static library and headers.

- `raspberrypi/pico-sdk`  
  Used for Pico 2 W hardware support, CYW43 Wi-Fi, lwIP, PWM, and the C/C++ SDK.

## What was intentionally not imported from live-movement

The old live-movement branch contained its own Wi-Fi, UDP, and micro-ROS setup. That code was not used because it did not work well over the network.

Do not copy these from the old branch into this firmware:

~~~text
old interface/wifi code
old interface/uart code
old interface/interface.h transport setup
old ROS initialization code
old UDP transport implementation
old CMakeLists.txt
old libmicroros folder
old main.c as a complete replacement
~~~

Only movement-related logic is intended to come from the old branch.

## Tested setup

Build machine:

~~~text
WSL2 Ubuntu 22.04
Pico SDK 2.2.0
PICO_BOARD=pico2_w
ROS 2 Humble libmicroros
~~~

Runtime host:

~~~text
Proxmox Docker/LXC host
micro-ROS Agent Humble container
ROS Humble container
~~~

Target board:

~~~text
Raspberry Pi Pico 2 W
~~~

## Network layout

Local LAN test:

~~~text
Pico 2 W
  -> Wi-Fi LAN
  -> micro-ROS Agent host UDP port 8888
  -> ROS 2 Humble graph
~~~

Internet test:

~~~text
Pico 2 W
  -> phone hotspot / external network
  -> public home IP
  -> router UDP port forward 8888
  -> micro-ROS Agent container
  -> ROS 2 Humble graph
~~~

For production use, prefer a VPN such as WireGuard instead of exposing the micro-ROS Agent directly on public UDP.

## Build-time configuration

The firmware should be built with Wi-Fi and Agent values passed through CMake.

Example LAN build:

~~~bash
cd ~/TINLAS_Robots_live_movement_integration/pico2w_humble_posearray_wifi

rm -rf build
mkdir build
cd build

cmake .. \
  -DPICO_BOARD=pico2_w \
  -DWIFI_SSID='YOUR_WIFI_SSID' \
  -DWIFI_PASSWORD='YOUR_WIFI_PASSWORD' \
  -DAGENT_IP='192.168.1.219' \
  -DAGENT_PORT=8888

make -j$(nproc)
~~~

Example internet test build:

~~~bash
cmake .. \
  -DPICO_BOARD=pico2_w \
  -DWIFI_SSID='YOUR_HOTSPOT_SSID' \
  -DWIFI_PASSWORD='YOUR_HOTSPOT_PASSWORD' \
  -DAGENT_IP='YOUR_PUBLIC_HOME_IP' \
  -DAGENT_PORT=8888
~~~

Do not commit real Wi-Fi credentials or private/public IPs.

## Install build dependencies on WSL2 Ubuntu 22.04

~~~bash
sudo apt update

sudo apt install -y \
  git \
  cmake \
  make \
  g++ \
  gcc-arm-none-eabi \
  libnewlib-arm-none-eabi \
  build-essential \
  python3
~~~

## Install Raspberry Pi Pico SDK

~~~bash
mkdir -p ~/pico
cd ~/pico

git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk

git checkout 2.2.0
git submodule update --init --recursive
~~~

Set `PICO_SDK_PATH`:

~~~bash
echo 'export PICO_SDK_PATH=$HOME/pico/pico-sdk' >> ~/.bashrc
source ~/.bashrc
~~~

Verify:

~~~bash
ls $PICO_SDK_PATH/external/pico_sdk_import.cmake
ls $PICO_SDK_PATH/src/boards/include/boards/pico2_w.h
~~~

## Add Humble libmicroros locally

This branch does not commit `libmicroros/`. Copy it locally before building.

~~~bash
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src

git clone -b humble https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git
~~~

Copy `libmicroros` into this firmware folder:

~~~bash
cp -r ~/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk/libmicroros \
  ~/TINLAS_Robots_live_movement_integration/pico2w_humble_posearray_wifi/libmicroros
~~~

Verify:

~~~bash
ls ~/TINLAS_Robots_live_movement_integration/pico2w_humble_posearray_wifi/libmicroros/libmicroros.a
ls ~/TINLAS_Robots_live_movement_integration/pico2w_humble_posearray_wifi/libmicroros/include/rcl/rcl.h
~~~

## Build firmware

~~~bash
cd ~/TINLAS_Robots_live_movement_integration/pico2w_humble_posearray_wifi

rm -rf build
mkdir build
cd build

cmake .. \
  -DPICO_BOARD=pico2_w \
  -DWIFI_SSID='YOUR_WIFI_SSID' \
  -DWIFI_PASSWORD='YOUR_WIFI_PASSWORD' \
  -DAGENT_IP='YOUR_AGENT_IP' \
  -DAGENT_PORT=8888

make -j$(nproc)
~~~

Expected output:

~~~text
build/pico_micro_ros_example.uf2
~~~

## Flash the Pico 2 W

Hold `BOOTSEL`, plug in the Pico 2 W, then copy:

~~~text
build/pico_micro_ros_example.uf2
~~~

to the Pico 2 W mass-storage drive.

From Windows Explorer, the WSL path may look like:

~~~text
\\wsl$\Ubuntu\home\<user>\TINLAS_Robots_live_movement_integration\pico2w_humble_posearray_wifi\build\pico_micro_ros_example.uf2
~~~

Adjust the WSL distro name and username if needed.

## Run the micro-ROS Agent

On the Docker host:

~~~bash
docker stop uros-agent 2>/dev/null || true

docker run --rm -it \
  --name uros-agent \
  --net=host \
  microros/micro-ros-agent:humble udp4 --port 8888
~~~

For verbose debugging:

~~~bash
docker run --rm -it \
  --name uros-agent \
  --net=host \
  microros/micro-ros-agent:humble udp4 --port 8888 -v6
~~~

## Open a ROS Humble shell

In another terminal on the Docker host:

~~~bash
docker run --rm -it --net=host ros:humble-ros-base bash
~~~

Inside the container:

~~~bash
source /opt/ros/humble/setup.bash
~~~

## Verify `/pico_counter`

~~~bash
ros2 topic list -t
ros2 topic echo /pico_counter
~~~

Expected:

~~~text
data: 0
---
data: 1
---
data: 2
~~~

## Publish PoseArray position updates

Use `--wait-matching-subscriptions 1` so the ROS CLI waits until the Pico subscriber is matched before publishing. Without this, the first few messages can be missed while discovery/matching completes.

One message:

~~~bash
ros2 topic pub --once --wait-matching-subscriptions 1 /robots/pos geometry_msgs/msg/PoseArray "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  poses: [
    {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  ]
}"
~~~

Continuous 10 Hz test:

~~~bash
ros2 topic pub --wait-matching-subscriptions 1 -r 10 /robots/pos geometry_msgs/msg/PoseArray "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  poses: [
    {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  ]
}"
~~~

Since `robot_num` is currently `0`, the firmware uses:

~~~text
poses[0]
~~~

as this robot's current pose.

## Movement behavior

The firmware initializes the motor PWM pins from `movement.h`:

~~~text
PWM_LM = GPIO 6
PWM_RM = GPIO 7
EN_L   = GPIO 11
EN_R   = GPIO 14
~~~

The target pose is currently hardcoded to:

~~~text
x = 0.0
y = 0.0
yaw = 0.0
~~~

When a valid `PoseArray` is received and `poses[0]` exists, the main loop calls:

~~~c
move_to(&all_robot_positions.poses.data[robot_num], &target);
~~~

The imported movement code then decides whether to:

~~~text
stop
turn left
turn right
move forward
~~~

based on distance to the target and heading error.

## Movement debug output

The movement code prints debug messages when the movement decision changes and then repeats the current decision once per second.

Example:

~~~text
[MOVE] turn left: distance=1.000 yaw=0.00 target_angle=180.00 delta=180.00 DEADZONE=10.00
[MOVE] move forward: distance=0.850 yaw=175.00 target_angle=180.00 delta=5.00 within DEADZONE=10.00
[MOVE] stop: target reached, distance=0.040 <= MIN_DISTANCE=0.050
~~~

## Serial monitor

On the machine connected to the Pico:

~~~bash
picocom /dev/ttyACM0 -b 115200
~~~

The firmware prints PoseArray receive messages and movement debug messages.

## Notes

- The working Wi-Fi/UDP/micro-ROS transport is kept from the Pico 2 W Humble PoseArray branch.
- Only movement functionality was imported from the old live-movement branch.
- The subscriber uses best-effort QoS.
- The counter publisher uses best-effort QoS.
- The default PoseArray topic is `/robots/pos`.
- The default Agent port is UDP `8888`.
- The firmware disables CYW43 power-save mode for more consistent Wi-Fi latency.
- The current target pose is hardcoded to `(0, 0, 0)`.
- The current robot index is hardcoded to `robot_num = 0`.

## Troubleshooting

### Pico joins Wi-Fi but Agent sees nothing

Check the compiled Agent IP and port values.

Rebuild with the correct values:

~~~bash
cmake .. \
  -DPICO_BOARD=pico2_w \
  -DWIFI_SSID='YOUR_WIFI_SSID' \
  -DWIFI_PASSWORD='YOUR_WIFI_PASSWORD' \
  -DAGENT_IP='YOUR_AGENT_IP' \
  -DAGENT_PORT=8888
~~~

Run the Agent with verbose logs:

~~~bash
docker run --rm -it --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
~~~

### First few PoseArray messages are missed

Use:

~~~bash
--wait-matching-subscriptions 1
~~~

with `ros2 topic pub`.

### Build cannot find `rcl/rcl.h` or `geometry_msgs/msg/pose.h`

Make sure local `libmicroros` exists:

~~~bash
ls libmicroros/libmicroros.a
ls libmicroros/include/rcl/rcl.h
ls libmicroros/include/geometry_msgs/msg/pose.h
~~~

If missing, copy the Humble `libmicroros` folder again.

### Build cannot find `movement.h`

Make sure the movement code is in:

~~~text
robot/movement.c
robot/movement.h
~~~

and that `CMakeLists.txt` includes:

~~~text
robot/movement.c
${CMAKE_CURRENT_LIST_DIR}/robot
~~~

### Motor does not move

Check PWM outputs:

~~~text
GPIO 6 = left motor PWM
GPIO 7 = right motor PWM
GND    = common ground
~~~

Neutral/stop should be around a 1.5 ms pulse at 50 Hz.

## micro-ROS reconnect fixes

This firmware is intended to handle Pico reboot/reconnect cycles better than the default micro-ROS setup.

Two fixes are used:

~~~text
1. Hard Liveliness Check in the micro-ROS Client library
2. A fixed micro-ROS Client Key in the firmware
~~~

### Why this is needed

When the Pico 2 W is rebooted, unplugged, or reflashed, the micro-ROS Agent and ROS 2 graph may temporarily keep the old node alive.

This can make `ros2 node list` show multiple stale `pico_node` entries after several Pico reboots.

The two fixes below reduce that problem:

- Hard Liveliness lets the Agent detect that the Pico client is gone and remove its entities.
- A fixed Client Key makes the Pico identify itself consistently across reboots.

## Hard Liveliness Check

Hard Liveliness Check is not enabled only by changing the firmware source. It must be compiled into `libmicroros`.

The firmware project does not commit `libmicroros/`, so this is a local build step.

### Build custom Humble libmicroros with Hard Liveliness

Clone a clean Humble micro-ROS Pico SDK repo:

~~~bash
cd ~/micro_ros_ws/src

rm -rf micro_ros_raspberrypi_pico_sdk_hard_liveliness

git clone -b humble https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git micro_ros_raspberrypi_pico_sdk_hard_liveliness

cd ~/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk_hard_liveliness
~~~

Patch `colcon.meta`:

~~~bash
python3 - <<'PY'
from pathlib import Path
import json

p = Path("microros_static_library/library_generation/colcon.meta")
data = json.loads(p.read_text())

args = data["names"]["microxrcedds_client"]["cmake-args"]

for flag in [
    "-DUCLIENT_HARD_LIVELINESS_CHECK=ON",
    "-DUCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT=1000",
]:
    if flag not in args:
        args.append(flag)

p.write_text(json.dumps(data, indent=2) + "\n")
PY
~~~

Verify:

~~~bash
grep -n "HARD_LIVELINESS" microros_static_library/library_generation/colcon.meta
~~~

Expected:

~~~text
-DUCLIENT_HARD_LIVELINESS_CHECK=ON
-DUCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT=1000
~~~

Make the builder script executable:

~~~bash
chmod +x microros_static_library/library_generation/library_generation.sh
~~~

Run the builder:

~~~bash
docker run --rm \
  -v $(pwd):/project \
  microros/micro_ros_static_library_builder:humble
~~~

Verify output:

~~~bash
ls libmicroros/libmicroros.a
ls libmicroros/include/rcl/rcl.h
ls libmicroros/include/rmw_microros/rmw_microros.h
~~~

Copy the custom `libmicroros` into this firmware folder:

~~~bash
cd ~/TINLAS_Robots_live_movement_integration/pico2w_humble_posearray_wifi

rm -rf libmicroros

cp -r ~/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk_hard_liveliness/libmicroros .
~~~

Do not commit `libmicroros/`; it is intentionally ignored by Git.

### Hard Liveliness timeout

The current recommended timeout is:

~~~text
1000 ms
~~~

If the Pico is falsely removed during normal Wi-Fi jitter, rebuild the custom `libmicroros` with:

~~~text
-DUCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT=2000
~~~

## Fixed micro-ROS Client Key

The firmware should use a fixed Client Key so the Pico identifies itself consistently to the micro-ROS Agent.

The key is configured at build time:

~~~bash
-DMICRO_ROS_CLIENT_KEY=0xC0FFEE01
~~~

Use a unique key per robot:

~~~text
robot 0: 0xC0FFEE01
robot 1: 0xC0FFEE02
robot 2: 0xC0FFEE03
robot 3: 0xC0FFEE04
robot 4: 0xC0FFEE05
~~~

Do not run multiple robots with the same Client Key at the same time.

## Build with fixed Client Key

Example full build:

~~~bash
cd ~/TINLAS_Robots_live_movement_integration/pico2w_humble_posearray_wifi

rm -rf build
mkdir build
cd build

cmake .. \
  -DPICO_BOARD=pico2_w \
  -DWIFI_SSID='YOUR_WIFI_SSID' \
  -DWIFI_PASSWORD='YOUR_WIFI_PASSWORD' \
  -DAGENT_IP='YOUR_AGENT_IP' \
  -DAGENT_PORT=8888 \
  -DMICRO_ROS_CLIENT_KEY=0xC0FFEE01

make -j$(nproc)
~~~

## Testing reconnect behavior

Start the Agent with verbose logging:

~~~bash
docker stop uros-agent 2>/dev/null || true

docker run --rm -it \
  --name uros-agent \
  --net=host \
  microros/micro-ros-agent:humble udp4 --port 8888 -v6
~~~

In another terminal, watch the ROS graph:

~~~bash
docker run --rm -it --net=host ros:humble-ros-base bash
~~~

Inside the container:

~~~bash
source /opt/ros/humble/setup.bash

watch -n 0.5 ros2 node list
~~~

Now reboot or unplug the Pico.

Expected behavior:

~~~text
The stale pico_node should disappear after roughly the configured Hard Liveliness timeout.
When the Pico boots again, it should reconnect using the same fixed Client Key.
Repeated Pico reboots should no longer accumulate many ghost pico_node entries.
~~~

