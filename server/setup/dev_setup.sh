# top-level script to setup everything needed for a dev environment

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
    echo "Running mac installer"
elif [ ${machine} = "Linux" ]; then
    echo "Running linux installer"
    echo "I'm going to ask for your su password to install postgres..."
    sudo apt-get update
    sudo apt-get install postgresql postgresql-contrib
else 
    echo "Sorry, I dont know how to properly install the server on your type of machine"
    exit 1
fi

# Python connector for postgres
# this assumes that you're environment is currently set to the python with ros installed
# (this may not be the case if you're using conda or venv)
echo "Install postgres python connector"
pip install psycopg2-binary