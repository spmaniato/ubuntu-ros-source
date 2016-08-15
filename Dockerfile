FROM ubuntu:trusty

MAINTAINER Spyros Maniatopoulos spmaniato@gmail.com

ENV DEBIAN_FRONTEND noninteractive
ENV TERM xterm

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# ROS distribution and configuration
ENV ROS_DISTRO indigo
ENV ROS_CONFIG ros_comm
ENV CATKIN_WS /usr/catkin_ws
ENV ROS_INSTALL_DIR /opt/ros/$ROS_DISTRO

RUN apt-get update && apt-get install -yq --no-install-recommends \
  build-essential \
  python-pip

# Install ROS-related Python tools
WORKDIR /usr
COPY ./requirements.txt .
RUN pip install -r requirements.txt

RUN rosdep init \
    && rosdep update

RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS

RUN rosinstall_generator $ROS_CONFIG --rosdistro $ROS_DISTRO \
    --deps --tar > .rosinstall \
    && wstool init src .rosinstall \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
       --skip-keys python-rosdep \
       --skip-keys python-rospkg \
       --skip-keys python-catkin-pkg

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

RUN mkdir -p $ROS_INSTALL_DIR

RUN catkin init \
    && catkin config --install --install-space $ROS_INSTALL_DIR \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build

# TODO: Delete tars in src after installation

COPY ./ros_entrypoint.sh .

ENTRYPOINT ["bash", "ros_entrypoint.sh"]

CMD ["bash"]
