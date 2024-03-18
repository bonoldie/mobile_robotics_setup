#!/bin/bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
/workspace/UnityHub.AppImage --no-sandbox -- --headless
exec "$@"