from classify.predictor import Predictor
import os, cv2

classes = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
           'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 
           'U', 'V', 'W', 'X', 'Y', 'Z']

class LetterPredictor(Predictor):

    def __init__(self):
        super(LetterPredictor, self).__init__(classes, 
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 
                'assets', 
                'nets',
                'letter-imaging-squeezenet.pt'))

if __name__ == "__main__":
    # test code to see if the predictor even works
    image = cv2.imread(os.path.join(os.path.dirname(os.path.realpath(__file__)),
        'assets',
        'img',
        'A.jpg'))

    predictor = LetterPredictor()
    result = predictor.predict(image, True)
    print(result)
