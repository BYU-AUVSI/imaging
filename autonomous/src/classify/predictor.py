import PIL# import Image
import torch
import cv2
import torch.nn as nn
from torchvision import transforms
from torch.autograd import Variable

input_size = 224
loader = transforms.Compose([
    transforms.Resize(input_size),
    transforms.CenterCrop(input_size),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])


class Predictor():

    def __init__(self, outputClasses, modelName):
        self.classes = outputClasses

        self.model = torch.load(modelName, map_location='cpu')
        self.model.eval()


    def predict(self, cvImg, verbose=False):
        """
            @ptype cvImg: openCV image
            @param cvImg: 200x200px openCV image

            @rtype: string
            @returns: String of the classification it found after running the given image through the net
        """
        loadedImg = self.loadImage(cvImg)
        with torch.set_grad_enabled(False):
            outputs = self.model(loadedImg)

            if verbose:
                print(outputs)

            # whichever output class has the highest confidence is our answer
            #   this is a pretty simple way todo it for now. could do more 
            #   based on how far above the confidence is from other classes
            _, preds = torch.max(outputs, 1)
            answer = self.classes[preds[0]].lower()
            return answer

    def loadImage(self, cvImg):
        """
        Takes a CV image, applies necessary transforms, returns an image ready
        to go through the predictor
        """
        cvImg = cv2.cvtColor(cvImg, cv2.COLOR_BGR2RGB)
        pilImg = PIL.Image.fromarray(cvImg)
        pilImg = pilImg.convert('RGB')
        img = loader(pilImg).float()
        img = Variable(torch.Tensor(img), requires_grad=False)
        img = img.unsqueeze(0)
        img = img.to('cpu')
        return img