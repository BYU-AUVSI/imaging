echo "======================================="
echo "NOTE: For a development setup, it is HIGHLY recommended"
echo "that you use an environment manager such as conda or virtualenv"
echo "======================================="

read -p "Install using conda? (Y/N): " confirm

if [ $confirm = "Y" -o $confirm = "y" ]; then
    conda env create -f conda-requirements.yaml
    while read requirement; do conda install --yes $requirement; done < pip-requirements.txt
elif [ $confirm = "N" -o $confirm = "n" ]; then
    pip3 install -r pip-requirements.txt
else
    echo "Sorry, Im a stupid script who doesn't understand anything beyond exactly 'Y' or 'N'"
fi