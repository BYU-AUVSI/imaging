
REPO=${PWD}/../..
# docker build  tagname path pathToDockerFile
docker build -t byuauvsi/imaging_server ${REPO} -f ${PWD}/Dockerfile
# run the server interactive tty will give it control of the terminal you run it in
# so that it can print output --publish forwards the containers port 5000 to your port 5000
docker run --interactive --tty \
  -e ROS_MASTER_URI="$ROS_MASTER_URI" \
  --publish 5000:5000 \
  --name imaging-server byuauvsi/imaging_server

  # --network host 
  #       only works on linux.. no mac or windows support. which defeats the purpose here