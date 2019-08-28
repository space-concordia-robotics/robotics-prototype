from flask import Flask, url_for, request, render_template

app = Flask(__name__)

@app.route('/')
def index():
    return 'Index Page'

@app.route('/hello')
def hello_world():
    helo = 'Hello, World!'
    helo += '<br />URL is: ' #returns html script to server
    helo += url_for('hello_world')
    return helo#'Hello, World!'

@app.route('/login', methods=['GET','POST'])
def login():
    if request.method == 'POST':
        return 'you posted'
    else:
        return 'you\'re getting<button>a button</button>'

@app.route('/<name>')
def ros(name=None):
    return render_template('rosweb.html', name=name)

with app.test_request_context():
    print(url_for('hello_world')) #prints address based on function callback
