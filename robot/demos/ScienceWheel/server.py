from flask import Flask
import time

app = Flask(__name__)

@app.route('/')
def index():
    return open('index.html').read()

@app.route('/index.html')
def index2():
    return open('index.html').read()

@app.route('/numSections')
def numSections():
    return '13'

@app.route('/initialSection')
def initialSection():
    return '0'

@app.route('/rotatePos')
def rotatePos():
    time.sleep(1) # wait 1 second
    return 'done'

@app.route('/rotateNeg')
def rotateNeg():
    time.sleep(1)
    return 'done'

app.run('0.0.0.0', 8000)
