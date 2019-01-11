
# detect machine
unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=Mac;;
    CYGWIN*)    machine=Cygwin;;
    MINGW*)     machine=MinGw;;
    *)          machine="UNKNOWN:${unameOut}"
esac


if [ ${machine} = "Mac" ]; then
    echo "==========================="
    echo "NOTE: At the time of this scripts creation, Docker is unable to"
    echo "share the network of the host Mac, meaning it can be very finiky when it comes"
    echo "to attaching to a ROS network on LAN (necessary for the ros_ingestor to "
    echo "subscribe to topics and get imageing data). Unfortunately the best solution at the moment"
    echo "is to simply run the docker image on Linux. We'll start it for you anyway's, in the event"
    echo "you're just looking to use the REST server"
    echo "==========================="
    # run the server interactive tty will give it control of the terminal you run it in
    # so that it can print output --publish forwards the containers port 5000 to your port 5000
    # Note: Perhaps there's a way to EXPOSE/Publish all ports from the container onto a host machine
    #       to get ROS working?? You do have to expose a whole lot of ports though since ROS kinda randomly
    #       chooses ports to use when subscribing to a topic
    docker run --interactive --tty \
        -e ROS_MASTER_URI="$ROS_MASTER_URI" \
        --publish 5000:5000 \
        --name imaging-server byuauvsi/imaging_server
elif [ ${machine} = "Linux" ]; then
    echo "==========================="
    echo "NOTE: this image will use your current ROS_MASTER_URI"
    echo "==========================="
    # run the server interactive tty will give it control of the terminal you run it in
    # so that it can print output --publish forwards the containers port 5000 to your port 5000
    # --network host  -> only works on linux.. no mac or windows support. which defeats the 
    # purpose here (of cross platform functioning server), but it allows the image to share 
    # the linux host's network, which allows for more robust ros connection. Basically without ROS
    # all the other pieces of the server (REST and database) would work perfectly on their own 
    # network
    docker run --interactive --tty \
        --network host \
        -e ROS_MASTER_URI="$ROS_MASTER_URI" \
        --publish 5000:5000 \
        --name imaging-server byuauvsi/imaging_server
else 
    echo "Sorry, I dont know how to properly run docker on your type of machine (probably the same as the mac command??)"
    exit 1
fi