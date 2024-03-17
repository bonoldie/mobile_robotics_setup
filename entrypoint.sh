#!/bin/bash
export TURTLEBOT3_MODEL=burger
/workspace/UnityHub.AppImage --no-sandbox -- --headless
exec "$@"