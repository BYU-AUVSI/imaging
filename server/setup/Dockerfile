#===============================
# SECTION 1:: BASE DEPENDENCIES:
#===============================
FROM ros:kinetic-ros-core-xenial

#########################################
#   If you wanna upgrade to melodic, you
#   need to change the core container
#   (in the above FROM) and the base
#   package retrieval in the below RUN
#########################################

# install ros kinetic base packages (core is handled above)
# this is pulled from osrf/docker_images
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-base=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

# install database and python3 for the server
RUN apt-get update -y && \
    apt-get install -y \
    postgresql \
    postgresql-contrib \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

#===============================
# SECTION 2: ROS DEPENDENCIES
#===============================
# prep the catkin ws:
RUN mkdir -p /opt/catkin_ws/src 
WORKDIR /opt/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_init_workspace"

# Get package dependencies. we need the State message from ROSPlane
# we really dont care or want to compile the rest of rosplane, 
# so remove those folders until we have just rosplane_msgs left
RUN git clone https://github.com/BYU-AUVSI/rosplane.git rosplane && \
    cd rosplane && \
    rm -rf rosplane rosplane_sim

# Get package dependencies. We need the GPS message from 
# the inertial sense in order to do geolocation and intake gps data
RUN git clone https://github.com/BYU-AUVSI/inertial_sense_ros.git ins && \
    cd ins && \
    git submodule update --init --recursive


#===============================
# SECTION 3: PYTHON3 DEPENDENCIES
#===============================


#===============================
# SECTION 4: STARTUP
#===============================
ENTRYPOINT [ "bash" ]
# ENTRYPOINT [ "python3" ]
# CMD [ "--version" ]