from classify.predictor import Predictor

classes = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
           'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 
           'U', 'V', 'W', 'X', 'Y', 'Z']

class LetterPredictor(Predictor):

    def __init__(self):
        super(LetterPredictor, self).__init__(classes, 'assets/letter-imaging-squeezenet.pt')