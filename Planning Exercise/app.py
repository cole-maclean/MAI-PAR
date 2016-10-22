"""
This file is part of the flask+d3 Hello World project.
"""
import json
import flask
from flask import request
import numpy as np


app = flask.Flask(__name__)


@app.route("/")
def index():
    """
    When you request the index path, you'll get the index.html template.
    """
    robot_cell = request.args.get('robot_cell', '')
    if len(robot_cell)==0: robot_cell="1"
    return flask.render_template("index.html",robot_cell=robot_cell,machines="",petitions="")

@app.route("/runplan")
def runplan():
    robot_cell = request.args.get('robot_cell', '')
    machines = request.args.get('machines', '').split(",")
    petitions = request.args.get('petitions', '').split(",")
    print (machines)
    if len(robot_cell)==0: robot_cell="1"
    return flask.render_template("runplan.html")

if __name__ == "__main__":
    import os

    port = 8000

    # Open a web browser pointing at the app.
    os.system("open http://localhost:{0}/".format(port))

    # Set up the development server on port 8000.
    app.debug = True
    app.run(port=port)