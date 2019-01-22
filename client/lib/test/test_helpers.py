import requests

def resetManualDb():
    resp = requests.get("http://127.0.0.1:5000/util/reset/manual")
    assert resp.status_code == 200