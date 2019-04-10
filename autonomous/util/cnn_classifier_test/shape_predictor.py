import torch
import torch.nn as nn
import argparse
import PIL# import Image
import os
from torchvision import transforms
from torch.autograd import Variable

classes = ['circle',
    'cross',
    'heptagon',
    'hexagon',
    'notarget',
    'octagon',
    'pentagon',
    'quarterCircle',
    'rectangle',
    'semicircle',
    'square',
    'star',
    'trapezoid',
    'triangle']

input_size = 224
loader = transforms.Compose([
    transforms.Resize(input_size),
    transforms.CenterCrop(input_size),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])

def predict(imgPath):
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

def loadImage(imgPath):
    pilImg = PIL.Image.open(imgPath)
    pilImg = pilImg.convert('RGB')
    img = loader(pilImg).float()
    img = Variable(torch.Tensor(img), requires_grad=False)
    img = img.unsqueeze(0)
    img = img.to('cpu')
    return img

parser = argparse.ArgumentParser(description="Shape classifier predictor")
parser.add_argument("image_to_classify", help="Relative path to an image or folder of images to classify")
parser.add_argument("-v", action='store_true', help="Verbose mode - print out class prediction outputs for an image")
parser.add_argument("-f", action="store_false", help="Print out prediction results for all images in a folder, not just failures")
args = parser.parse_args()

model = torch.load('shape-imaging-squeezenet.pt', map_location='cpu')
model.eval()

if os.path.isdir(args.image_to_classify):
    # we assume there are only images in the provided folder
    images = sorted(os.listdir(args.image_to_classify))
    success = 0
    attempts = 0
    for imagePath in images:
        if imagePath.endswith(".jpg") or imagePath.endswith(".jpeg") or imagePath.endswith(".png"):
            attempts += 1
            result = predict(os.path.join(args.image_to_classify, imagePath))
            success =  success + 1 if result else success
    print("============\nFINAL RESULT = {} / {}\n============".format(success, attempts))
else:
    predict(args.image_to_classify)