# Robots

Firmware for the physical Chariot robots used in the Jachtseizoen robot swarm. The firmware runs on a Raspberry Pi Pico W, connects to Wi-Fi, joins the ROS 2 system through micro-ROS, receives game and position data, and drives the robot as runner or hunter.

## Features

- micro-ROS UDP connection to the project ROS 2 graph.
- Ready-state publication after boot and reset.
- Game command handling for start, pause, resume and reset.
- Runner and hunter behaviour based on the selected runner ID.
- Repulsive force-field movement with wall and robot avoidance.
- Differential wheel control for the Chariot platform.
- Local safety stop when own position updates are missing.
- Watchdog reset safety halt when all position updates are lost.

## Repository contents

| Path | Purpose |
|---|---|
| `src/main.c` | Firmware entrypoint, Wi-Fi setup, micro-ROS setup, callbacks and runtime loop. |
| `src/settings.h` | Robot identity, movement, field and topic configuration. |
| `src/robot/rff-algorithm.c` | Repulsive force-field pathfinding. |
| `src/robot/movement.c` | Differential movement and motor PWM control. |
| `src/picow_udp_transports.c` | Pico W UDP transport for micro-ROS. |
| `secrets.h.template` | Template for local Wi-Fi and Agent credentials. |
| `setup.cmake` | Fetches dependencies and builds the firmware. |
| `docker-compose.yaml` | Local micro-ROS Agent and ROS 2 test environment. |
| `test/` | Local test code. |

## Getting started

### Requirements

- Raspberry Pi Pico W
- Chariot robot platform
- CMake
- `arm-none-eabi-gcc`
- Git
- Docker and Docker Compose for local micro-ROS Agent testing
- Access to a running micro-ROS Agent in the final system

### Install and build

1. Clone the repository.
2. Create the local secrets file:

```bash
cp secrets.h.template secrets.h
```

3. Fill in the Wi-Fi and micro-ROS Agent values in `secrets.h`.
4. Build the firmware:

```bash
cmake -P setup.cmake
```

The script downloads the Pico SDK, the precompiled micro-ROS Pico assets and picotool into `external/`, then builds the firmware into `build/src/`.

5. Flash the generated UF2 file:

```bash
external/picotool/build/picotool load -f -x build/src/main.uf2
```

Alternatively, hold BOOTSEL while connecting the Pico W and copy `build/src/main.uf2` to the mounted Pico storage device.

6. Open a serial monitor to view firmware logs.

### Local micro-ROS Agent for bench testing

The final deployment uses the server micro-ROS Agent. For local testing, this repository also includes a compose service:

```bash
docker compose up -d micro-ros-agent
```

## Configuration

### Local secrets

`secrets.h` is not committed. It must define:

| Macro | Recommended value | Effect |
|---|---|---|
| `WIFI_SSID` | Project Wi-Fi SSID | Wi-Fi network used by the robot. |
| `WIFI_PASSWORD` | Project Wi-Fi password | Wi-Fi credential. |
| `AGENT_IP` | Server or Agent IP address | micro-ROS Agent endpoint. |
| `AGENT_PORT` | `8888` | micro-ROS Agent UDP port. |
| `MICRO_ROS_CLIENT_KEY` | Unique integer per robot | micro-ROS client key. |

### Firmware settings

The final values are defined in `src/settings.h`.

| Setting | Current value | Effect |
|---|---:|---|
| `TAG_NUM` | `103` | Logical robot ID for this firmware build. Change per physical robot. |
| `MAX_ROBOTS_IN_GAME` | `10` | Maximum number of poses stored from the position array. |
| `POS_TOPIC` | `/robots/pos` | Position input. |
| `ROS_READY_TOPIC` | `/game/robots/ready` | Ready-state output. |
| `ROS_COMMAND_TOPIC` | `/game/command` | Game command input. |
| `ROS_SEEN_TOPIC` | `/robots/seen` | Runner visibility input. |
| `FIELD_MIN_X`, `FIELD_MAX_X` | `0.0`, `10.0` | Field X limits in position-system units. |
| `FIELD_MIN_Y`, `FIELD_MAX_Y` | `0.0`, `10.0` | Field Y limits in position-system units. |
| `MOVE_SPEED_RPS` | `0.6` | Forward movement speed. |
| `MAX_TURNING_RPS` | `0.6` | Maximum turning speed. |
| `MIN_TURNING_RPS` | `0.05` | Minimum turning speed. |
| `DEADZONE` | `10.0` | Angle deadzone before turning is required. |
| `MIN_MOVE_DISTANCE` | `0.3` | Distance at which a target is considered reached. |
| `MAX_MILLIS_WITHOUT_NEW_POSITION` | `500` | Stop when own pose is missing longer than this. |
| `MAX_MILLIS_WITHOUT_ANY_POSITION` | `5000` | Safety halt/reset when all pose updates are missing longer than this. |
| `HUNTER_FLEE_RADIUS` | `3.5` | Runner starts fleeing when a hunter is within this radius. |
| `CORNER_CHANGE_INTERVAL_MS` | `10000` | Runner patrol target change interval. |
| `WALL_SAFETY_MARGIN` | `1.2` | Distance from wall where wall repulsion starts. |
| `K_GOAL` | `2.0` | Goal attraction force. |
| `K_ROBOT_REPULSION` | `7.0` | Robot repulsion force. |
| `K_WALL` | `10.0` | Wall repulsion force. |
| `K_RUNNER_FLEE` | `4.5` | Runner flee force. |
| `K_RAND` | `0.7` | Random jitter force. |

## Actions

### Hardware actions

| Action | Method |
|---|---|
| Flash firmware | Hold BOOTSEL while connecting the Pico W, then copy the UF2 file or use picotool. |
| Start robot | Power the robot after flashing. It connects to Wi-Fi and the micro-ROS Agent automatically. |
| View logs | Connect a serial monitor to the Pico W USB serial port. |

### Runtime commands received by the robot

| Command | Effect |
|---|---|
| `start <runner_id>` | Starts the round. If the robot is not the runner, it waits two seconds before running. |
| `pause` | Stops motors and enters paused state. |
| `resume` | Returns to running state. |
| `reset` | Stops movement, clears local game state and publishes ready again. |

## Calibration

There is no runtime calibration step for the robot firmware. Correct behaviour depends on:

- the correct `TAG_NUM` for the physical AprilTag on the robot;
- correct Wi-Fi and micro-ROS Agent settings in `secrets.h`;
- a calibrated Tracking Module publishing the robot's position;
- movement constants in `src/settings.h` matching the physical field and robot hardware.

Changing robot identity, field limits or movement constants requires rebuilding and flashing the firmware.

## Connections

| Direction | Interface | Purpose |
|---|---|---|
| Outgoing | Wi-Fi | Connects to the network that can reach the micro-ROS Agent. |
| Outgoing | micro-ROS UDP | Connects the Pico W firmware to the ROS 2 graph. |
| Incoming | `/robots/pos` (`geometry_msgs/msg/PoseArray`) | All known robot positions. |
| Incoming | `/game/command` (`std_msgs/msg/String`) | Start, pause, resume and reset commands. |
| Incoming | `/robots/seen` (`std_msgs/msg/Bool`) | Whether hunters can chase the runner. |
| Outgoing | `/game/robots/ready` (`std_msgs/msg/Int32`) | Publishes this robot's ID after boot and reset. |
| Hardware | PWM motor outputs | Drives the left and right motors. |
| Hardware | Encoder GPIO inputs | Reads wheel encoder ticks for movement support. |
