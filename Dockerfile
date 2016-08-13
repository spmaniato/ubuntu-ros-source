FROM ubuntu:trusty

MAINTAINER Spyros Maniatopoulos spmaniato@gmail.com

# Resin base images already include these two environment variables
ENV DEBIAN_FRONTEND noninteractive
ENV TERM xterm

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# ROS distribution and configuration
ENV ROS_DISTRO indigo
ENV ROS_CONFIG ros_comm
ENV ROS_HOME /home/ros
ENV CATKIN_WS $ROS_HOME/catkin_ws
ENV ROS_TOOLS "python-rosdep python-rosinstall-generator python-rosinstall python-catkin-tools"

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116

RUN apt-get update && apt-get install -yq --no-install-recommends \
  build-essential \
  $ROS_TOOLS \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

RUN rosdep init

# Now create the ros user and set its permissions
RUN adduser --gecos "ROS User" --disabled-password ros
RUN usermod -a -G dialout ros

ADD 99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Switch to this new user
USER ros

# Update the package list (necessary for upcoming rosdep install to work)
RUN sudo apt-get update

# HOME needs to be set explicitly. Without it, the HOME environment variable is
# set to "/"
RUN HOME=$ROS_HOME rosdep update

RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS

RUN rosinstall_generator $ROS_CONFIG --rosdistro $ROS_DISTRO \
    --deps --tar > .rosinstall \
    && wstool init src .rosinstall \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

RUN catkin init \
    && catkin config --install --default-install-space \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build

COPY ./ros_entrypoint.sh $ROS_HOME
WORKDIR $ROS_HOME
RUN sudo chown ros:ros ros_entrypoint.sh

ENTRYPOINT ["bash", "ros_entrypoint.sh"]

CMD ["bash"]
