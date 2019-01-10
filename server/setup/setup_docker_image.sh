echo "========================"
echo "Hi there. I'm going to build the docker image for you."
echo "========================"
REPO=${PWD}/../..
# docker build  tagname path pathToDockerFile
docker build -t byuauvsi/imaging_server ${REPO} -f ${PWD}/Dockerfile


echo "DONE! with the docker image"

# detect machine
unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=Mac;;
    CYGWIN*)    machine=Cygwin;;
    MINGW*)     machine=MinGw;;
    *)          machine="UNKNOWN:${unameOut}"
esac

if [ ${machine} = "Linux" ]; then
    echo "Looks like you're running linux. Neat!"
    echo "I could edit your crontab so that this docker image"
    echo "starts automatically when the machine boots."
    echo ""
    read -p "Edit crontab? (Y/N)" confirm
    if [ $confirm = "Y" -o $confirm = "y" ]; then
        # copy command into crontab and set it to run @reboot
        echo "@reboot docker run --network host --interactive --tty -e ROS_MASTER_URI=\"\$ROS_MASTER_URI\" --publish 5000:5000 --name imaging-server byuauvsi/imaging_server" | sudo tee -a /var/spool/cron/crontabs/$USER
    fi
fi
