# Readme

## MicroROS Agent Docker command:
```
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

*Note: Make sure you're on the same network and your firewall doesn't block port 8888*

## Testing ROS Node --> MicroROS Agent --> Pico 2W
Used for testing whether messages come through to the Pico with the MicroROS agent.

1. Change "ssid" and "pass" to the appropriate wifi access point's ssid and password
2. Search up your PC's IP address and change "agent_ip"
3. Compile Pico code & flash it. The Pico should disconnect and reconnect
```bash
cd wifi_uros_pico_test/build
cmake -DWIFI_SSID={YOUR_SSID} -DWIFI_PASSWORD={YOUR_PASSWORD} -DAGENT_IP={YOUR_AGENT_IP} ..
make
# Set PICO in boot mode and connect with USB, reconnect if it was already connected
cp main.uf2 /run/media/$USER/RP2350/  # Change to the corrects mountpoint of the pico
```
5. Open a Serial Monitor and keep listening on the pico's serial. If it says "failed to connect", reconnect the pico (without setting it in boot mode)
4. Run the MicroROS Agent:
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```
5. Run a regular ROS Node:
```bash
docker run -it --rm --net=host ros:humble
```
6. Within the previous ROS Node, check whether the "pico_0/pos" topic exists:
```bash
ros2 topic list
```
7. Send a PoseArray to the pico's topic, it should recognize 2 Poses and print in the serial monitor: *"Received message on the POSITION Topic!Received 2 poses. Robot 0 is at: (1.20, 3.40)"*
```bash
ros2 topic pub --once /pico_0/pos geometry_msgs/msg/PoseArray "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  poses: [
    {position: {x: 1.2, y: 3.4, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    {position: {x: 5.6, y: 7.8, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  ]
}"
```





