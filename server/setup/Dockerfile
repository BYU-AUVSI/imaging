#===============================
# SECTION 1:: BASE DEPENDENCIES:
#===============================
FROM ros:kinetic-ros-core-xenial

#########################################
#   If you wanna upgrade to melodic, you
#   need to change the core container
#   (in the above FROM) and the base
#   package retrieval (in the below RUN)
#   Moving up to ubuntu 18.04 could
#   also potentially simplify the rest 
#   of the image. Currently have to do a
#   bunch of extra stuff in order to 
#   install postgres11 and python3.7.
#   16.04 has older version of both of
#   these by default
#########################################

# install ros kinetic base packages (core is handled above)
# this is pulled from osrf/docker_images
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-base=1.3.2-0* \
    ros-kinetic-image-transport \
    ros-kinetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# install database and python3 for the server
RUN apt-get update -y && \
    apt-get install -y \
    # pip 2 for ros
    python-pip \
    python3 \
    python3-pip \
    # supervisor allows us to start multiple services in the background with single docker run command
    supervisor \
    # for postgres11
    wget ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# setup postgres repository. By default ubuntu 16 apt has
# postgres 9.5, but we are using 11 - the latest version.
RUN wget --quiet -O - https://www.postgresql.org/media/keys/ACCC4CF8.asc | apt-key add - \
    && /bin/sh -c 'echo "deb http://apt.postgresql.org/pub/repos/apt/ `lsb_release -cs`-pgdg main" >> /etc/apt/sources.list.d/pgdg.list' \
    && apt-get update \
    && apt-get install -y postgresql postgresql-contrib

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

# Get package dependencies. The InteropImage msg from uav_msgs
# is used to transport targets in their final form to the interop_pkg
# for final submission to judges
RUN git clone https://github.com/BYU-AUVSI/uav_msgs.git uav_msgs

# copy over our actual server code
COPY server /opt/catkin_ws/src/server

# build all the ROS stuff
WORKDIR /opt/catkin_ws
SHELL [ "/bin/bash", "-c" ]
RUN source /opt/ros/kinetic/setup.bash && \
    catkin_make

#===============================
# SECTION 3: SERVER SETUP
#===============================
# get python3 dependencies
WORKDIR /opt/catkin_ws/src/server/setup
RUN pip3 install -r internal/pip-requirements.txt
RUN pip install -r internal/pip-requirements.txt

# run our postgres setup script
USER postgres
RUN echo "There will likely be 4-5 errors about the `imaging_server` role not existing. This is OK. For more info on these errors see server/setup/internal/setup_database.sql"
RUN /etc/init.d/postgresql start \
    && psql -f /opt/catkin_ws/src/server/setup/internal/setup_database.sql

#===============================
# SECTION 4: STARTUP
#===============================
USER root
COPY server/setup/internal/supervisor.conf /etc/supervisor/conf.d/supervisord.conf
WORKDIR /opt/catkin_ws/src/server/src
CMD ["/opt/catkin_ws/src/server/setup/internal/docker_startup.sh"]