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

# installing python3.7 on ubuntu 16.04
# is a bit of a pain, but can be done with this stuff:
# wholllle ton of lib dependencies for python3.7
# RUN apt-get install -y \
#     python3-dev \
#     python3-setuptools zlib1g-dev libbz2-dev libc6-dev libgdbm-dev tk-dev libsqlite3-dev libssl-dev libncursesw5-dev libreadline-gplv2-dev openssl libffi-dev
# RUN mkdir -p /tmp/python37
# WORKDIR /tmp/python37
# RUN wget https://www.python.org/ftp/python/3.7.0/Python-3.7.0.tar.xz && \
#     tar xvf Python-3.7.0.tar.xz && \
#     cd Python-3.7.0 && \
#     ./configure && \
#     make altinstall && \
#     pip3.7 install -U pip

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
# RUN pip3.7 install -r pip-requirements.txt
RUN pip3 install -r pip-requirements.txt
RUN pip install -r pip-requirements.txt

# run our postgres setup script
USER postgres
RUN /etc/init.d/postgresql start \
    && psql -f /opt/catkin_ws/src/server/setup/setup_database.sql

# copy supervisor configuration files over to the supervisor folder

COPY server/setup/internal/server_supervisor.conf /etc/supervisor/conf.d/server_supervisor.conf

#===============================
# SECTION 4: STARTUP
#===============================
# TODO: need to source catkin_ws and start ros_ingestor
USER root
WORKDIR /opt/catkin_ws/src/server/src
ENTRYPOINT [ "bash" ]
# CMD [ "python3", "/opt/catkin_ws/src/server/src/server.py", "--host", "0.0.0.0" ]