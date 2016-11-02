"""
This file is part of the flask+d3 Hello World project.
"""
import json
import flask
from flask import request
from Planner import Planner


def parse_input_state(robot_cell,petitions,machines):
    parsed_petitions = {i:int(petition_count) for i,petition_count in enumerate(petitions) if int(petition_count) > 0}
    parsed_machines = {i:int(machine_count) for i,machine_count in enumerate(machines) if int(machine_count) > 0}
    return {'robot-location':int(robot_cell),'robot-free':True,'robot-loaded':0,'petitions':parsed_petitions,'served':[],'machines':parsed_machines,'steps':0}

app = flask.Flask(__name__)


@app.route("/")
def index():
    """
    When you request the index path, you'll get the index.html template.
    """
    robot_cell = request.args.get('robot_cell', '')
    if len(robot_cell)==0: robot_cell="0"
    return flask.render_template("index.html",robot_cell=robot_cell,machines="",petitions="")

@app.route("/runplan")
def runplan():
    robot_cell = request.args.get('robot_cell','')
    petitions = request.args.get('petitions','')
    machines = request.args.get('machines','')
    return flask.render_template("runplan.html",robot_cell=robot_cell,machines=machines,petitions=petitions)

@app.route("/gdata")
def gdata():
    robot_cell = request.args.get('robot_cell','')
    petitions = request.args.get('petitions','').split(',')
    machines = request.args.get('machines','').split(',')
    parsed_petitions = {i:int(petition_count) for i,petition_count in enumerate(petitions) if int(petition_count) > 0}
    parsed_machines = {i:int(machine_count) for i,machine_count in enumerate(machines) if int(machine_count) > 0}
    initial_state = {'robot-location':int(robot_cell),'robot-free':True,'robot-loaded':0,'petitions':parsed_petitions,'served':[],'machines':parsed_machines,'steps':0}
    plan = Planner(initial_state,36).build_plan()
    return json.dumps(plan)


if __name__ == "__main__":
    import os

    port = 8000

    # Open a web browser pointing at the app.
    os.system("open http://localhost:{0}/".format(port))

    # Set up the development server on port 8000.
    app.debug = True
    app.run(port=port)



