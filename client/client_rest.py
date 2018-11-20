import requests
from PIL import Image
from io import BytesIO

def getNextRawImage():
    print("eyyyyyy")
    img = requests.get('http://192.168.1.11:5000/image/raw', headers={'X-Manual': 'True'})
    print("response code:: {}".format(img.status_code))
    return Image.open(BytesIO(img.content))


if __name__ == "__main__":
    getNextRawImage()
