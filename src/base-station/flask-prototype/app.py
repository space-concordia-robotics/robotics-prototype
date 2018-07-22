# This is the controller page of the Flask GUI
# Flask is light-weight and modular so this is actually all we need to set up a simple HTML page

import flask

app = flask.Flask(__name__)


# Once we launch this, this will route us to the "../" page or index page and automatically render the Rover GUI
@app.route("/")
def index():
    return flask.render_template("Rover_GUI.html")

@app.route("/BaseContainers")
def base_containers():
	return flask.render_template("BaseContainers.html")

@app.route("/BaseContainers2")
def base_containers2():
	return flask.render_template("BaseContainers2.html")

app.run(debug=True) # add param `host= '0.0.0.0'` if you want to run on your machine's IP address
