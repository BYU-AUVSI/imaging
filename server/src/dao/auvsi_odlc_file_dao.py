import json, os, shutil
from config import defaultSubmittedImgDir

class AuvsiOdlcDao():
    """
    Handles creating the AUVSI Object File Format folder as described in the AUVSI rules.
    This folder is used to submit targets if for whatever reason, network submission to 
    judges fails
    """

    def addTarget(self, outgoingTarget, imgPath):
        """ 
        Adds the given target classification to the ODLC backup folder.

        This folder stores all the targets that we attempt to submit to the judges
        in the ODLC target format as described in the rules PDF for AUVSI.
        The final structure looks something like this:

        images
          |--1546372042
              |--submitted
                   |--manual
                       |--1.json
                       |--1.jpg
                       |--2.json
                       |--2.jpg
                    |--autonomous
                       |--1.json
                       |--1.jpg
                       |--3.json
                       |--3.jpg
        """
        # get the path we'll be saving to:
        baseSubmittedPath = defaultSubmittedImgDir()
        if outgoingTarget.autonomous:
            baseSubmittedPath = os.path.join(baseSubmittedPath, 'autonomous')
        else:
            baseSubmittedPath = os.path.join(baseSubmittedPath, 'manual')
        
        if not os.path.exists(baseSubmittedPath):
            os.makedirs(baseSubmittedPath)
            
        jsonFileOut = os.path.join(baseSubmittedPath, str(outgoingTarget.target) + '.json')

        extension = imgPath.rsplit('.', 1)[1].lower()
        fileName  = imgPath.rsplit('/', 1)[1]
        newFileName = str(outgoingTarget.target) + '.' + extension

        # write target info to dictionary
        with open(jsonFileOut, 'w') as f:
            json.dump(outgoingTarget.toJson(), f)

        # copy over
        shutil.copy(imgPath, baseSubmittedPath)
        
        # rename file
        dst_file = os.path.join(baseSubmittedPath, fileName)
        new_dst_file_name = os.path.join(baseSubmittedPath, newFileName)
        os.rename(dst_file, new_dst_file_name)
        return True
