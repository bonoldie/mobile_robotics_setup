#!/bin/bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=33
/workspace/UnityHub.AppImage --no-sandbox -- --headless
exec "$@"