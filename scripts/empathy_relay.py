#!/usr/bin/env python

from flask import Flask
import flask
import rospy
from bystander_intervention_study.msg import(
    EmpathyGoal,
    EmpathyAction
)
from std_msgs.msg import Bool
import actionlib


name = "empathy_relay"
rospy.init_node(name)

client = actionlib.SimpleActionClient('empathy_channel',EmpathyAction)
publisher = rospy.Publisher('kill_switch',Bool,queue_size=10)
app = Flask(name)

@app.route("/kill_switch")
def kill():
    msg = Bool()
    msg.data = True
    publisher.publish(msg)
    response = flask.jsonify({'response': True})
    response.headers.add('Access-Control-Allow-Origin', '*')
    print("return response")
    return response

@app.route("/empathy_callback/<user>/<response>")
def hello(user,response):

    rospy.loginfo("answer:player{}:response:{}".format(user, response))

    print("here")
    goal = EmpathyGoal()
    goal.answer = response
    client.send_goal_and_wait(goal)
    result = client.get_result()
    
    response = flask.jsonify({'response': result.response})
    response.headers.add('Access-Control-Allow-Origin', '*')
    print("return response")
    return response

    #return "{}".format(result.response)

@app.route("/")
def hello2():
    return "Hello 2"


app.run(host='0.0.0.0')