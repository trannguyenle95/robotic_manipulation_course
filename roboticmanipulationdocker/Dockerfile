
# ros distribution to base on
FROM osrf/ros:melodic-desktop-full

# ---- CHANGE PACKAGE NAMES HERE IF NEEDED ----
# these should be without any .tar.gz endings

ARG FRANKA_ROS=franka_ros-manipulation_course
ARG LUMI_TESTBED=lumi_testbed-master
#ARG LUMI_TESTBED=lumi_testbed-two_robots
ARG MUJOCO_ROS_CONTROL=mujoco_ros_control-master

# ---------------------------------------------


# arg variable for using noninetractive apt
ARG DEBIAN_FRONTEND=noninteractive

# update packages
# install wget and unzip (mandatory)
# vim and tmux (optional)
# and then the libraries required by the course tasks
RUN apt update \
	&& apt install -y \
	wget \
	unzip \
	vim \
	tmux \
	ros-melodic-libfranka \
	ros-melodic-franka-ros \
	ros-melodic-joint-state-publisher-gui \
	ros-melodic-moveit \
	ros-melodic-moveit-visual-tools

# --- MUJOCO (Pro) 2.0 installation ---
WORKDIR /root/.mujoco
RUN wget https://www.roboti.us/download/mujoco200_linux.zip \
	&& unzip mujoco200_linux.zip \
	&& rm -rf mujoco200_linux.zip \
	&& mv mujoco200_linux mjpro200

# copy the license key to the required places
COPY mjkey.txt /root/.mujoco/mjkey.txt
COPY mjkey.txt /root/.mujoco/mjpro200/mjkey.txt
COPY mjkey.txt /root/.mujoco/mjpro200/bin/mjkey.txt

# create the catkin workspace tree
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
# -------------------------------------


# -- ROS packages & Catkin workspace --

# copy the ros projects needed for the course into the catkin workspace tree
COPY $LUMI_TESTBED.tar.gz /root/catkin_ws/src/$LUMI_TESTBED.tar.gz
COPY $MUJOCO_ROS_CONTROL.tar.gz /root/catkin_ws/src/$MUJOCO_ROS_CONTROL.tar.gz
COPY $FRANKA_ROS.tar.gz /root/catkin_ws/src/$FRANKA_ROS.tar.gz
	
# unpack the provided projects and remove compressed files
RUN cd /root/catkin_ws/src \
	&& tar -xf $LUMI_TESTBED.tar.gz \
	&& tar -xf $FRANKA_ROS.tar.gz \
	&& tar -xf $MUJOCO_ROS_CONTROL.tar.gz \
	&& rm -rf $LUMI_TESTBED.tar.gz \
			$FRANKA_ROS.tar.gz \
			$MUJOCO_ROS_CONTROL-master.tar.gz

# source setup.bash for correct paths and run catkin_make in the workspace root
# this will install the ros projects above
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && cd /root/catkin_ws && catkin_make"
# -------------------------------------

# --- Container start configuration ---

# copy the entrypoint.sh, this is the script that runs when the container is started
# the default version starts the roscore and leaves the container up and running
# if you edit the contents of this file, you need to build the image again before they are in effect
COPY entrypoint.sh /entrypoint.sh

# the default entry point, this will be overrided if you run the container with custom commands
CMD ["bash", "/entrypoint.sh"]
