
FROM ros:noetic-ros-base-focal

WORKDIR /ws

RUN apt-get update && apt-get install -y bash

RUN sudo apt update
RUN sudo apt install -y software-properties-common


ENV DISPLAY=:0


CMD [ "bash", "-c", "sleep infinity" ]

