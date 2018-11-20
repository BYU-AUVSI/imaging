unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=Mac;;
    CYGWIN*)    machine=Cygwin;;
    MINGW*)     machine=MinGw;;
    *)          machine="UNKNOWN:${unameOut}"
esac

echo "======================================="
echo "NOTE: For a development setup, it is HIGHLY recommended"
echo "that you use an environment manager such as conda or virtualenv"
echo "======================================="

read -p "Install using conda? (Y/N): " confirm

if [ $confirm = "Y" -o $confirm = "y" ]; then
    if [ ${machine} = "Mac" ]; then
        conda env create -f conda-reqs-mac.yaml
    elif [ ${machine} = "Linux" ]; then
        conda env create -f conda-reqs-linux.yaml
    else 
        echo "Sorry, I dont know how to properly install the dev stuff on your type of machine"
        exit 1
    fi
    conda env create -f conda-requirements.yaml
elif [ $confirm = "N" -o $confirm = "n" ]; then
    pip3 install -r pip-requirements.txt
else
    echo "Sorry, Im a stupid script who doesn't understand anything beyond exactly 'Y' or 'N'"
fi