#!/usr/bin/python
from flask import Flask
from flask import request
from flask import render_template
import logging
import rospy as rp
from std_msgs.msg import String

app = Flask(__name__)
constantUpdatePub = None

from constants import Constants
CONSTANTS_FILE_NAME = 'constants.yaml'
constants = Constants(CONSTANTS_FILE_NAME)

@app.route('/')
def index():
    keyValuePairs = constants.getAll().items()
    return render_template('index.html', constants=keyValuePairs)

@app.route('/update', methods=['POST'])
def update():
    for key, value in dict(request.form).items():
        value = value[0]
        constants.set(key, value)
    constantUpdatePub.publish('')
    return ''

def main():
    global constantUpdatePub
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(debug=False, host='0.0.0.0')
    rp.init_node('constants_server', anonymous=False)
    constantUpdatePub = rp.Publisher('constantUpdate', String, queue_size=10)

if __name__ == '__main__':
    try:
        main()
    except rp.ROSInterruptException:
        pass
