import PIL# import Image
import torch
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


    def predict(self, imgPath):
        truth = imgPath.split(".")[-2].split("/")[-1] # we get the truth letter from the file name
        truth = ''.join([i for i in truth if not i.isdigit()])
        loadedImg = loadImage(imgPath)
        with torch.set_grad_enabled(False):
            outputs = model(loadedImg)

            if args.v:
                print(outputs)

            _, preds = torch.max(outputs, 1)
            ans = classes[preds[0]].lower()
            result = truth.lower() == ans
            if (args.f and not result) or not args.f:
                print('{:30} == {:<15}'.format(imgPath, ans))
            return result

    def loadImage(self, imgPath):
        pilImg = PIL.Image.open(imgPath)
        pilImg = pilImg.convert('RGB')
        img = loader(pilImg).float()
        img = Variable(torch.Tensor(img), requires_grad=False)
        img = img.unsqueeze(0)
        img = img.to('cpu')
        return img