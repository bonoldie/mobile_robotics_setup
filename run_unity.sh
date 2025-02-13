#!/bin/bash
# Setup the X server
xhost +local:

# Run the contianer
docker run -it \
    --entrypoint /entrypoint_unity.sh \
    --privileged \
    --pid host \
    --net host \
    --cap-add SYS_ADMIN \
    --security-opt apparmor:unconfined \
    --device /dev/fuse:rw \
    --env "DISPLAY=$DISPLAY" \
    -v /dev/shm:/dev/shm \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /opt/unity:/opt/unity \
    -v /opt/unityhub:/opt/unityhub \
    -v ./.config/unityhub:/root/.config/unityhub \
    -v ./Turtlebot3UnityROS2:/root/ws/Turtlebot3UnityROS2 \
    -v ./DEFAULT_FASTRTPS_PROFILES.xml:/root/ws/DEFAULT_FASTRTPS_PROFILES.xml \
    -p 5678:5678/tcp \
    mobile-robotics 
#   -p 12400-12431:12400-12431/udp \

