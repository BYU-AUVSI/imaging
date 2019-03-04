echo "========================"
echo "Hi there. I'm going to build the docker image for you."
echo "========================"
REPO=${PWD}/../..
# docker build  tagname path pathToDockerFile
docker build -t byuauvsi/imaging_server ${REPO} -f ${PWD}/Dockerfile


echo "DONE! Building the docker image"