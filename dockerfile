FROM ubuntu:20.04
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update 

# ROS2 domain_id 30 ports interval
EXPOSE 12400-12431
EXPOSE 11811/udp

# Workspace setup
RUN mkdir /workspace
COPY ./Installation /workspace/Installation

WORKDIR /workspace
RUN chmod u+x ./Installation/*.sh

# Run installers
RUN ./Installation/install_mono.sh
RUN ./Installation/install_ros2_foxy.sh
RUN ./Installation/turtlebot3_nodes.sh
RUN ./Installation/install_unityHub.sh

RUN ln -s /usr/lib/x86_64-linux-gnu/libpython3.8.so.1.0 /usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0

ADD entrypoint_unity.sh /entrypoint_unity.sh
ADD entrypoint_container.sh /entrypoint_container.sh
RUN chmod u+x /entrypoint_unity.sh /entrypoint_container.sh
ENTRYPOINT [ "/entrypoint_container.sh" ]