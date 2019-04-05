import torch
import torch.nn as nn
import argparse
import PIL# import Image
import os
from torchvision import transforms
from torch.autograd import Variable

classes = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z']

input_size = 224
loader = transforms.Compose([
    transforms.Resize(input_size),
    transforms.CenterCrop(input_size),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])

def loadImage(imgPath):
    pilImg = PIL.Image.open(imgPath)
    pilImg = pilImg.convert('RGB')
    img = loader(pilImg).float()
    img = Variable(torch.Tensor(img), requires_grad=False)
    img = img.unsqueeze(0)
    img = img.to('cpu')
    return img

parser = argparse.ArgumentParser(description="Letter Classifier Test")
parser.add_argument("image_to_classify", help="Relative path to an image or folder of images to classify")
args = parser.parse_args()

model = torch.load('letter-imaging-squeezenet.pt', map_location='cpu')

model.eval()
print("Model loaded and set to eval mode")

if os.path.isdir(args.image_to_classify):
    # we assume there are only images in the provided folder
    images = sorted(os.listdir(args.image_to_classify))
    for imagePath in images:

        loadedImg = loadImage(os.path.join(args.image_to_classify, imagePath))
        with torch.set_grad_enabled(False):
            outputs = model(loadedImg)

            _, preds = torch.max(outputs, 1)
            print('{}\t== {}'.format(imagePath, classes[preds[0]]))
else:
    loadedImg = loadImage(args.image_to_classify)
    print('Loaded image!')
    with torch.set_grad_enabled(False):
        outputs = model(loadedImg)

        print(outputs)

        _, preds = torch.max(outputs, 1)
        print('Prediction == {}'.format(classes[preds[0]]))