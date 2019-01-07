import os, time
from configparser import ConfigParser
import time;
ts = time.time()

ALLOWED_EXTENSIONS = set(['jpg', 'jpeg', 'png'])

def allowedFileType(filename):
    """
    Check that the given filename has an extension that we allow
    """
    # make sure:
    #   1) the filename even has a . to begin with
    #   2) stuff the the right of the furthest . (rsplit) is one of our allowed extensions
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

def defaultConfigPath():
    return os.path.abspath('../conf/config.ini')

def getLatestBaseImgDir():
    # root image dir:
    rootImgDir = os.path.abspath('../images/')
    if not os.path.isdir(rootImgDir):
        os.makedirs(rootImgDir)
    latestSubDir = rootImgDir

    imgDirs = [d for d in os.listdir(rootImgDir) if os.path.isdir(os.path.join(rootImgDir, d))]
    imgDirs = [os.path.join(rootImgDir, dirname) for dirname in imgDirs]
    print(f'IMG DIRSSS::: {imgDirs}')
    if not imgDirs:
        # if the directory is empty, create a folder with the current timestamp
        ts = str(int(time.time())) + '/'
        os.makedirs(os.path.join(rootImgDir, ts))
        latestSubDir = os.path.join(latestSubDir, ts)
    else:
        # get the newest (aka largest unix time) folder
        latestSubDir = max(imgDirs, key=os.path.getmtime)
    return latestSubDir

#TODO: still not totally sold on this...
def defaultCroppedImgPath():
    latestSubDir = getLatestBaseImgDir()
    latestSubDir = os.path.join(latestSubDir, 'crop')
    if not os.path.exists(latestSubDir):
        os.makedirs(latestSubDir)
    return latestSubDir

def defaultRawImgPath():
    latestSubDir = getLatestBaseImgDir()
    latestSubDir = os.path.join(latestSubDir, 'raw')
    if not os.path.exists(latestSubDir):
        os.makedirs(latestSubDir)
    return latestSubDir
 
def defaultSubmittedImgDir():
    latestSubDir = getLatestBaseImgDir()
    latestSubDir = os.path.join(latestSubDir, 'submitted')
    if not os.path.exists(latestSubDir):
        os.makedirs(latestSubDir)
    return latestSubDir

def config(filename='config.ini', section='postgresql'):
    parser = ConfigParser() # create parser
    parser.read(filename) # read config file
 
    # get section, default section is postgresql
    sectionRet = {}
    if parser.has_section(section):
        params = parser.items(section)
        for param in params:
            sectionRet[param[0]] = param[1]
    else:
        raise Exception('Section {0} not found in the {1} file'.format(section, filename))
 
    return sectionRet
