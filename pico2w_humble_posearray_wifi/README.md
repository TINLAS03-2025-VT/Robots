# Pico 2 W micro-ROS Wi-Fi PoseArray Subscriber

This folder contains a Raspberry Pi Pico 2 W micro-ROS example for ROS 2 Humble.

It is based on the Pico 2 W Wi-Fi UDP micro-ROS example from:

<https://github.com/RIPLaboratoryUH/pico2w-microros-wifi-guide>

The original example publishes a counter over Wi-Fi using a custom UDP micro-ROS transport. This version keeps that transport and counter publisher, and adds a `geometry_msgs/msg/PoseArray` subscriber.

## Behavior

The Pico 2 W:

1. Connects to Wi-Fi.
2. Connects to a micro-ROS Agent over UDP.
3. Publishes an incrementing counter on `/pico_counter`.
4. Subscribes to `/robots/pos`.
5. Prints a short message over USB serial whenever a `PoseArray` is received.

Published topic:

```text
/pico_counter std_msgs/msg/UInt64
```

Subscribed topic:

```text
/robots/pos geometry_msgs/msg/PoseArray
```

Example serial output:

```text
posearray received 1, poses=2
posearray received 2, poses=2
posearray received 3, poses=2
```

## Credits

This work is based on:

- **RIPLaboratoryUH/pico2w-microros-wifi-guide**  
  Used as the Pico 2 W Wi-Fi UDP micro-ROS firmware base.

- **micro-ROS/micro_ros_raspberrypi_pico_sdk**  
  Used for the ROS 2 Humble `libmicroros` static library and headers.

- **raspberrypi/pico-sdk**  
  Used for Pico 2 W hardware support, CYW43 Wi-Fi, lwIP, and the C/C++ SDK.

## Tested setup

Firmware build machine:

```text
WSL2 Ubuntu 22.04
Pico SDK 2.2.0
PICO_BOARD=pico2_w
ROS 2 Humble libmicroros
```

Runtime host:

```text
Docker/LXC Linux host
micro-ROS Agent Humble container
ROS Humble container
```

Target board:

```text
Raspberry Pi Pico 2 W
```

## Network layout

```text
Pico 2 W
  -> Wi-Fi
  -> micro-ROS Agent host UDP port 8888
  -> ROS 2 Humble graph
```

Example:

```text
Agent IP: 192.168.1.219
Agent UDP port: 8888
```

## Required local configuration

Before building, edit:

```text
pico_micro_ros_example.c
```

Set:

```c
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";
```

Then edit:

```text
picow_udp_transports.h
```

Set:

```c
#define ROS_AGENT_IP_ADDR "YOUR_AGENT_IP"
#define ROS_AGENT_UDP_PORT 8888
```

Do not commit real Wi-Fi credentials or private/public IPs unless intentional.

## Install build dependencies on WSL2 Ubuntu 22.04

```bash
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
```

## Install Raspberry Pi Pico SDK

```bash
mkdir -p ~/pico
cd ~/pico

git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk

git checkout 2.2.0
git submodule update --init --recursive
```

Set `PICO_SDK_PATH`:

```bash
echo 'export PICO_SDK_PATH=$HOME/pico/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
ls $PICO_SDK_PATH/external/pico_sdk_import.cmake
ls $PICO_SDK_PATH/src/boards/include/boards/pico2_w.h
```

## Add Humble libmicroros

Clone the official Humble Pico micro-ROS repo:

```bash
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src

git clone -b humble https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git
```

Copy `libmicroros` into this firmware folder:

```bash
cp -r ~/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk/libmicroros \
  ~/TINLAS_Robots_posearray_branch/pico2w_humble_posearray_wifi/libmicroros
```

Verify:

```bash
ls ~/TINLAS_Robots_posearray_branch/pico2w_humble_posearray_wifi/libmicroros/libmicroros.a
ls ~/TINLAS_Robots_posearray_branch/pico2w_humble_posearray_wifi/libmicroros/include/rcl/rcl.h
```

## Build firmware

```bash
cd ~/TINLAS_Robots_posearray_branch/pico2w_humble_posearray_wifi

rm -rf build
mkdir build
cd build

cmake .. -DPICO_BOARD=pico2_w
make -j$(nproc)
```

Expected output:

```text
pico_micro_ros_example.uf2
```

## Flash the Pico 2 W

Hold `BOOTSEL`, plug in the Pico 2 W, then copy:

```text
build/pico_micro_ros_example.uf2
```

to the Pico 2 W mass-storage drive.

From Windows Explorer, the WSL path may look like:

```text
\\wsl$\Ubuntu\home\<user>\TINLAS_Robots_posearray_branch\pico2w_humble_posearray_wifi\build\pico_micro_ros_example.uf2
```

Adjust the WSL distro name and username if needed.

## Run the micro-ROS Agent

On the Linux Docker host:

```bash
docker stop uros-agent 2>/dev/null || true

docker run --rm -it \
  --name uros-agent \
  --net=host \
  microros/micro-ros-agent:humble udp4 --port 8888
```

For debugging:

```bash
docker run --rm -it \
  --name uros-agent \
  --net=host \
  microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

## Verify `/pico_counter`

In another shell on the Docker host:

```bash
docker run --rm -it --net=host ros:humble-ros-base bash
```

Inside:

```bash
source /opt/ros/humble/setup.bash

ros2 topic list -t
ros2 topic echo /pico_counter
```

Expected:

```text
data: 0
---
data: 1
---
data: 2
```

## Test the PoseArray subscriber

Inside the ROS Humble container:

```bash
source /opt/ros/humble/setup.bash

ros2 topic pub --once /robots/pos geometry_msgs/msg/PoseArray "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  poses: [
    {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    {position: {x: 3.0, y: 4.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}
  ]
}"
```

Expected Pico serial output:

```text
posearray received 1, poses=2
```

For repeated messages:

```bash
ros2 topic pub -r 2 /robots/pos geometry_msgs/msg/PoseArray "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  poses: [
    {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    {position: {x: 3.0, y: 4.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}
  ]
}"
```

## Serial monitor

On the machine connected to the Pico:

```bash
picocom /dev/ttyACM0 -b 115200
```

The firmware currently prints only when a `PoseArray` is received.

## Notes

- The existing `/pico_counter` publisher is kept.
- The custom UDP transport is kept from RIPLaboratoryUH’s Pico 2 W Wi-Fi micro-ROS example.
- The subscriber uses best-effort QoS.
- The default PoseArray topic is `/robots/pos`.
- The default Agent port is UDP `8888`.
- The firmware disables CYW43 power-save mode for more consistent latency.
- Do not expose the micro-ROS Agent publicly for production use without a VPN/firewall.

## Troubleshooting

### Pico joins Wi-Fi but the Agent sees nothing

Check:

```bash
grep -n "ROS_AGENT" picow_udp_transports.h
```

Make sure the IP is the micro-ROS Agent host IP and the port matches the Agent command.

Run the Agent with verbose logs:

```bash
docker run --rm -it --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

### Agent sees packets but ROS topic does not appear

Check inside a ROS Humble container:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list -t
```

Also confirm the firmware was built with the Humble `libmicroros` folder.

### Build error about `rclc_timer_t`

For Humble, use:

```c
rcl_timer_t timer;
```

not:

```c
rclc_timer_t timer;
```

### Wrong Agent port

The upstream example may use `8899`. This branch is intended to use `8888`.

Confirm:

```bash
grep -n "ROS_AGENT_UDP_PORT" picow_udp_transports.h
```

Expected:

```c
#define ROS_AGENT_UDP_PORT 8888
```
