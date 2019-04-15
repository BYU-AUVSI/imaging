from classify.predictor import Predictor
import os

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

class ShapePredictor(Predictor):


    def __init__(self):
        super(ShapePredictor, self).__init__(classes, 
            os.path.join(os.path.dirname(os.path.realpath(__file__)), # absolute path to this files folder
                'assets', 
                'letter-imaging-squeezenet.pt'))