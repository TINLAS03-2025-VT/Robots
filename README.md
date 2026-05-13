# Readme

## MicroROS Agent Docker command:
```
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

*Note: Make sure you're on the same network and your firewall doesn't block port 8888*

