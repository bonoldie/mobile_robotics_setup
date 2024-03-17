FROM ubuntu:20.04
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update 

# ROS2 domain_id 20 ports interval
EXPOSE 12400-12431
EXPOSE 11811/udp

# Workspace setup
RUN mkdir /workspace
COPY ./Installation /workspace/Installation
# COPY ./MobileRoboticsDQN /workspace/MobileRoboticsDQN
# COPY ./ROS2Package /workspace/ROS2Package
# COPY ./Turtlebot3UnityROS2 /workspace/Turtlebot3UnityROS2

WORKDIR /workspace
RUN chmod u+x ./Installation/*.sh

# Run installers
RUN ./Installation/install_mono.sh
RUN ./Installation/install_ros2_foxy.sh
RUN ./Installation/turtlebot3_nodes.sh
RUN ./Installation/install_unityHub.sh

RUN ln -s /usr/lib/x86_64-linux-gnu/libpython3.8.so.1.0 /usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0

ADD entrypoint.sh /entrypoint.sh
RUN chmod u+x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]