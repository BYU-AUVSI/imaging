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
    brew update
    brew install postgresql
    brew services start postgresql
    createdb `whoami`
    # run sql setup script to setup the database
    psql -f internal/setup_database.sql
elif [ ${machine} = "Linux" ]; then
    echo "Running linux installer"
    echo "I'm going to ask for your su password to install postgres..."
    sudo apt-get update
    sudo apt-get install postgresql postgresql-contrib
    # run sql setup script as the postgres user to setup auvsi database
    workDir=$PWD
    sudo -u postgres -H sh -c "cd $workDir; psql -f internal/setup_database.sql" 
else 
    # feel free to add Windows Cygwin/MinGW compatability if you want..
    echo "Sorry, I dont know how to properly install the server on your type of machine"
    exit 1
fi


echo "======================================="
echo "NOTE: For a development setup, it is HIGHLY recommended"
echo "that you use an environment manager such as conda or virtualenv"
echo "======================================="

read -p "Install using conda? (Y/N): " confirm

if [ $confirm = "Y" -o $confirm = "y" ]; then
    if [ ${machine} = "Mac" ]; then
        conda env create -f internal/server-conda-reqs-mac.yaml
    elif [ ${machine} = "Linux" ]; then
        conda env create -f internal/server-conda-reqs-mac.yaml
    else 
        echo "Sorry, I dont know how to properly install the dev stuff on your type of machine"
        exit 1
    fi
elif [ $confirm = "N" -o $confirm = "n" ]; then
    echo "Install pip modules"
    pip install -r internal/server-pip-requirements.txt
else
    echo "Sorry, Im a stupid script who doesn't understand anything beyond exactly 'Y' or 'N'"
fi
