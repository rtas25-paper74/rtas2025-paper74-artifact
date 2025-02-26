FROM osrf/ros:foxy-desktop

RUN apt update && apt install -y ninja-build clang-18

WORKDIR /root
ADD ros2_ws ros2_ws

WORKDIR /root/ros2_ws
RUN . /opt/ros/foxy/setup.sh && colcon build

CMD ["/bin/bash", "-C", "run_mode_switch_demo.sh"]
