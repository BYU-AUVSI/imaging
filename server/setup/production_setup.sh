
REPO=${PWD}/../..
# docker build  tagname path pathToDockerFile
docker build -t byuauvsi/imaging_server ${REPO} -f ${PWD}/Dockerfile
# run the server interactive tty will give it control of the terminal you run it in
# so that it can print output --publish forwards the containers port 5000 to your port 5000
docker run --interactive --tty --publish 5000:5000 --name imaging-server byuauvsi/imaging_server