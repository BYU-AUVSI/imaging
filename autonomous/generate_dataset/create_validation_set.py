import os
import random

# WARN:: This will move images around in your dataset. save a copy 
#          its current structure before executing just to be safe
# TODO: atm, this assumes that you have ONLY files in each train/classification directory. should make this safer
# the main idea of this script is given a directory to a training 
# set, you can generate a validation set with a specified 
# percentage of the training set
# directory structure is as specified for a torchvision ImageFolder eg:
#       dataset/train/class1/images.png
#                    /class2/images.png
#              /val/class1/images.png
#              /val/class2/images.png

# TODO: make these arguments:
DATASET_BASE_DIR_ = 'generated/'
VAL_PERCENTAGE_ = 0.15 # ie: 20% of all current images for a class will be moved to validation

def get_immediate_subdirectories(a_dir):
    return [name for name in os.listdir(a_dir)
            if os.path.isdir(os.path.join(a_dir, name))]

for classification in get_immediate_subdirectories(DATASET_BASE_DIR_ + 'train/'):
    
    trainDir = DATASET_BASE_DIR_ + 'train/' + classification + '/'
    valDir = DATASET_BASE_DIR_ + 'val/' + classification + '/'
    # create the validation class folder
    if not os.path.exists(valDir):
        os.makedirs(valDir)


    # files = 
    ttlNumInstances = len(os.listdir(trainDir))
    print(ttlNumInstances)
    numValInstances = int(ttlNumInstances * VAL_PERCENTAGE_)
    print("Moving {} random images for '{}' to validation".format(numValInstances, classification))
    for i in range(numValInstances):
        fileToMove = random.choice(os.listdir(trainDir))
        os.rename(trainDir + fileToMove, valDir + fileToMove)