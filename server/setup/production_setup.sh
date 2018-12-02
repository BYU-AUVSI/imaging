SERVER=${PWD}/..
REPO=${SERVER}/..
# docker build  tagname path pathToDockerFile
docker build -t byuauvsi/imaging_server ${REPO} -f ${PWD}/Dockerfile
#- -publish 5000:5000
docker run --interactive --tty --name imaging-server byuauvsi/imaging_server