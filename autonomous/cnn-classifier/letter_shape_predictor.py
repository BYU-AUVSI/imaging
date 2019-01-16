import torch
import torch.nn as nn
import PIL# import Image
from torchvision import transforms
from torch.autograd import Variable

alphabet = list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")

input_size = 224
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
loader = transforms.Compose([
    transforms.Grayscale(num_output_channels=3), # this needs to go first. expecting pil image
    transforms.Resize(input_size),
    transforms.CenterCrop(input_size),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])

def loadImage(imgPath):
    pilImg = PIL.Image.open(imgPath)
    img = loader(pilImg).float()
    img = Variable(img, requires_grad=False)
    img = img.unsqueeze(0)
    img = img.to(device)
    return img

model = torch.load('pretrained-squeezenet.pt')
model = model.to(device)
# model.classifier[1] = nn.Conv2d(512, 26, kernel_size=(1,1), stride=(1,1))
model.eval()
print("Model loaded and set to eval mode")

# loadedImg = loadImage('square_y.jpg')
loadedImg = loadImage('square_a.jpg')
print('Loaded image!')
with torch.set_grad_enabled(False):
    outputs = model(loadedImg)

    print(outputs)

    _, preds = torch.max(outputs, 1)
    print('Prediction == {}'.format(alphabet[preds[0]]))