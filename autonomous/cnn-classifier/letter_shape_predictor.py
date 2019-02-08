import torch
import torch.nn as nn
import PIL# import Image
from torchvision import transforms
from torch.autograd import Variable

classes = ['A-circle', 'A-cross', 'A-heptagon', 'A-hexagon', 'A-octagon', 'A-pentagon', 'A-quarterCircle', 'A-rectangle', 'A-semicircle', 'A-square', 'A-star', 'A-trapezoid', 'A-triangle',
    'B-circle', 'B-cross', 'B-heptagon', 'B-hexagon', 'B-octagon', 'B-pentagon', 'B-quarterCircle', 'B-rectangle', 'B-semicircle', 'B-square', 'B-star', 'B-trapezoid', 'B-triangle',
    'C-circle', 'C-cross', 'C-heptagon', 'C-hexagon', 'C-octagon', 'C-pentagon', 'C-quarterCircle', 'C-rectangle', 'C-semicircle', 'C-square', 'C-star', 'C-trapezoid', 'C-triangle',
    'D-circle', 'D-cross', 'D-heptagon', 'D-hexagon', 'D-octagon', 'D-pentagon', 'D-quarterCircle', 'D-rectangle', 'D-semicircle', 'D-square', 'D-star', 'D-trapezoid', 'D-triangle',
    'E-circle', 'E-cross', 'E-heptagon', 'E-hexagon', 'E-octagon', 'E-pentagon', 'E-quarterCircle', 'E-rectangle', 'E-semicircle', 'E-square', 'E-star', 'E-trapezoid', 'E-triangle',
    'F-circle', 'F-cross', 'F-heptagon', 'F-hexagon', 'F-octagon', 'F-pentagon', 'F-quarterCircle', 'F-rectangle', 'F-semicircle', 'F-square', 'F-star', 'F-trapezoid', 'F-triangle',
    'G-circle', 'G-cross', 'G-heptagon', 'G-hexagon', 'G-octagon', 'G-pentagon', 'G-quarterCircle', 'G-rectangle', 'G-semicircle', 'G-square', 'G-star', 'G-trapezoid', 'G-triangle',
    'H-circle', 'H-cross', 'H-heptagon', 'H-hexagon', 'H-octagon', 'H-pentagon', 'H-quarterCircle', 'H-rectangle', 'H-semicircle', 'H-square', 'H-star', 'H-trapezoid', 'H-triangle',
    'I-circle', 'I-cross', 'I-heptagon', 'I-hexagon', 'I-octagon', 'I-pentagon', 'I-quarterCircle', 'I-rectangle', 'I-semicircle', 'I-square', 'I-star', 'I-trapezoid', 'I-triangle',
    'J-circle', 'J-cross', 'J-heptagon', 'J-hexagon', 'J-octagon', 'J-pentagon', 'J-quarterCircle', 'J-rectangle', 'J-semicircle', 'J-square', 'J-star', 'J-trapezoid', 'J-triangle',
    'K-circle', 'K-cross', 'K-heptagon', 'K-hexagon', 'K-octagon', 'K-pentagon', 'K-quarterCircle', 'K-rectangle', 'K-semicircle', 'K-square', 'K-star', 'K-trapezoid', 'K-triangle',
    'L-circle', 'L-cross', 'L-heptagon', 'L-hexagon', 'L-octagon', 'L-pentagon', 'L-quarterCircle', 'L-rectangle', 'L-semicircle', 'L-square', 'L-star', 'L-trapezoid', 'L-triangle',
    'M-circle', 'M-cross', 'M-heptagon', 'M-hexagon', 'M-octagon', 'M-pentagon', 'M-quarterCircle', 'M-rectangle', 'M-semicircle', 'M-square', 'M-star', 'M-trapezoid', 'M-triangle',
    'N-circle', 'N-cross', 'N-heptagon', 'N-hexagon', 'N-octagon', 'N-pentagon', 'N-quarterCircle', 'N-rectangle', 'N-semicircle', 'N-square', 'N-star', 'N-trapezoid', 'N-triangle',    
    'O-circle', 'O-cross', 'O-heptagon', 'O-hexagon', 'O-octagon', 'O-pentagon', 'O-quarterCircle', 'O-rectangle', 'O-semicircle', 'O-square', 'O-star', 'O-trapezoid', 'O-triangle',
    'P-circle', 'P-cross', 'P-heptagon', 'P-hexagon', 'P-octagon', 'P-pentagon', 'P-quarterCircle', 'P-rectangle', 'P-semicircle', 'P-square', 'P-star', 'P-trapezoid', 'P-triangle',
    'Q-circle', 'Q-cross', 'Q-heptagon', 'Q-hexagon', 'Q-octagon', 'Q-pentagon', 'Q-quarterCircle', 'Q-rectangle', 'Q-semicircle', 'Q-square', 'Q-star', 'Q-trapezoid', 'Q-triangle',
    'R-circle', 'R-cross', 'R-heptagon', 'R-hexagon', 'R-octagon', 'R-pentagon', 'R-quarterCircle', 'R-rectangle', 'R-semicircle', 'R-square', 'R-star', 'R-trapezoid', 'R-triangle',
    'S-circle', 'S-cross', 'S-heptagon', 'S-hexagon', 'S-octagon', 'S-pentagon', 'S-quarterCircle', 'S-rectangle', 'S-semicircle', 'S-square', 'S-star', 'S-trapezoid', 'S-triangle',
    'T-circle', 'T-cross', 'T-heptagon', 'T-hexagon', 'T-octagon', 'T-pentagon', 'T-quarterCircle', 'T-rectangle', 'T-semicircle', 'T-square', 'T-star', 'T-trapezoid', 'T-triangle',
    'U-circle', 'U-cross', 'U-heptagon', 'U-hexagon', 'U-octagon', 'U-pentagon', 'U-quarterCircle', 'U-rectangle', 'U-semicircle', 'U-square', 'U-star', 'U-trapezoid', 'U-triangle',
    'V-circle', 'V-cross', 'V-heptagon', 'V-hexagon', 'V-octagon', 'V-pentagon', 'V-quarterCircle', 'V-rectangle', 'V-semicircle', 'V-square', 'V-star', 'V-trapezoid', 'V-triangle',
    'W-circle', 'W-cross', 'W-heptagon', 'W-hexagon', 'W-octagon', 'W-pentagon', 'W-quarterCircle', 'W-rectangle', 'W-semicircle', 'W-square', 'W-star', 'W-trapezoid', 'W-triangle',
    'X-circle', 'X-cross', 'X-heptagon', 'X-hexagon', 'X-octagon', 'X-pentagon', 'X-quarterCircle', 'X-rectangle', 'X-semicircle', 'X-square', 'X-star', 'X-trapezoid', 'X-triangle',
    'Y-circle', 'Y-cross', 'Y-heptagon', 'Y-hexagon', 'Y-octagon', 'Y-pentagon', 'Y-quarterCircle', 'Y-rectangle', 'Y-semicircle', 'Y-square', 'Y-star', 'Y-trapezoid', 'Y-triangle',
    'Z-circle', 'Z-cross', 'Z-heptagon', 'Z-hexagon', 'Z-octagon', 'Z-pentagon', 'Z-quarterCircle', 'Z-rectangle', 'Z-semicircle', 'Z-square', 'Z-star', 'Z-trapezoid', 'Z-triangle',
    'notarget']

input_size = 224
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
loader = transforms.Compose([
    transforms.Resize(input_size),
    transforms.CenterCrop(input_size),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])

def loadImage(imgPath):
    pilImg = PIL.Image.open(imgPath)
    img = loader(pilImg).float()
    img = Variable(torch.Tensor(img), requires_grad=False)
    img = img.unsqueeze(0)
    img = img.to(device)
    return img

model = torch.load('imaging-squeezenet.pt', map_location='cpu')

model.eval()
print("Model loaded and set to eval mode")

# loadedImg = loadImage('square_y.jpg')
loadedImg = loadImage('X-hexagon-rotate.jpg')
print('Loaded image!')
with torch.set_grad_enabled(False):
    outputs = model(loadedImg)

    print(outputs)

    _, preds = torch.max(outputs, 1)
    print('Prediction == {}'.format(classes[preds[0]]))