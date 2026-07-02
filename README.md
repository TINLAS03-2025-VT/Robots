# Readme

## MicroROS Agent Docker command:
To start a MicroROS agent on network port 8888, run this command:

```
docker compose up -d micro-ros-agent
```

*Note: For connecting to this agent, please make sure the MicroROS client (Raspberry Pi Pico (2) W) can reach the MicroROS agent.*

To start a MicroROS agent via a serial bus, run:
```
docker compose up -d --name microros_agent_serial micro-ros-agent serial --dev <Device Location> -v6
```

## Testing ROS Node --> MicroROS Agent --> Pico 2W
Used for testing whether messages come through to the Pico with the MicroROS agent. The commands should be run from the project root (`<Path To Repository>/Robots` by default).

*Warning: If you have not installed git, the arm-none-eabi toolchain or cmake, this will most likely not work. Make sure these are installed - or just FAAFO.*

1. Copy `secrets.h.template` to `secrets.h`. Change the placeholder values in `secrets.h` to the correct WiFi credentials and MicroROS agent IP.
2. In the root repository directory open the CMakeLists.txt and change the PICO_PLATFORM and PICO_BOARD (line 5 and 6) to the correct platform and board.
3. If the projects were built before and the Pico platform and board were changed, delete the build folder.
4. Setup the environment and compile the code by running `cmake -P setup.cmake`. The output of the compilation will be in `build/src`.
5. To flash the software, plug in the Pico in boot mode and copy the \<Software Name>.uf2 to it or, if the previous step has been done before, run: `external/picotool/build/picotool load -f -x build/src/main.uf2`
6. Open a serial monitor on the pico serial port. If it says "failed to connect", reconnect the pico (without setting it in boot mode)
7. Run the MicroROS agent:
```
docker compose up -d micro-ros-agent
```
8. Run the regular ROS node:
```
docker compose up --build -d ros2-test-env && docker compose attach ros2-test-env
```
9. Within the previous ROS node, check whether the "pico_0/pos" topic exists:
```
ros2 topic list
```
10. In the ROS node, send a PoseArray to the pico's topic, it should recognize 2 Poses and print in the serial monitor: "Received message on the POSITION Topic!Received 2 poses. Robot 0 is at: (1.20, 3.40)"  
```
ros2 topic pub --once /robots/pos geometry_msgs/msg/PoseArray "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  poses: [
    {position: {x: 1.2, y: 3.4, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    {position: {x: 5.6, y: 7.8, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  ]
}"
```
