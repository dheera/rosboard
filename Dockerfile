FROM ros:humble-ros-base

ARG USERNAME=arlyx
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ARG DISPLAY=:0

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get upgrade -y \
    && apt-get install -y python3-pip \
    && apt-get install -y curl \
    && apt-get install ros-humble-rviz2 -y

RUN pip3 install tornado simplejpeg rospkg

USER $USERNAME
WORKDIR /home/$USERNAME

COPY . /home/$USERNAME/

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source install/local_setup.bash" >> ~/.bashrc

EXPOSE 8888

ENV SHELL=/bin/bash
ENV DISPLAY=$DISPLAY

CMD ["./run"]