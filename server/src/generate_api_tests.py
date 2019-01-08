from flask import Flask, json
from apis import api
# import os

app = Flask(__name__)
api.init_app(app)

app.config['SERVER_NAME'] = 'localhost:5000'
with app.app_context():
    urlvars = False # Build query strings in URLs
    swagger = True # Export Swagger specifications
    data = api.as_postman(urlvars=urlvars, swagger=swagger)
    f = open('postman_import.json', 'w')
    f.write(json.dumps(data))