import os
from configparser import ConfigParser

ALLOWED_EXTENSIONS = set(['jpg', 'jpeg', 'png'])

def allowedFileType(filename):
    """
    Check that the given filename has an extension that we allow
    """
    # make sure:
    #   1) the filename even has a . to begin with
    #   2) stuff the the right of the furthest . (rsplit) is one of our allowed extensions
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

def defaultSqlConfigPath():
    return os.path.dirname(os.path.realpath(__file__))  + '/../conf/config.ini'

# TODO: this is crap
def defaultCroppedImgPath():
    return os.path.dirname(os.path.realpath(__file__)) +  '/../images/crop/'
 
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