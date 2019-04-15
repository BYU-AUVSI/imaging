from classify.predictor import Predictor

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
        super(ShapePredictor, self).__init__(classes, 'assets/shape-imaging-squeezenet.pt')