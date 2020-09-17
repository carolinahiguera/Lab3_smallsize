import sys
import time
import numpy as np

sys.path.insert(0, './proto')

import messages_robocup_ssl_detection_pb2
import messages_robocup_ssl_geometry_pb2
import messages_robocup_ssl_refbox_log_pb2
import messages_robocup_ssl_wrapper_pb2

ssl_fifo = None
ssl_wrapper = None

robotsInfo = None
ballsInfo = None


def init(maxRobots):
    global ssl_fifo, ssl_wrapper, robotsInfo
    ssl_fifo = open('/tmp/ssl_fifo', 'rb')
    ssl_wrapper = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
    robotsInfo = np.empty([2,maxRobots,3])
    robotsInfo[:] = np.nan


# dim 1: 0: blue, 1: yellow
# dim 2: robot id
# dim 3: 0: x, 1: y, 2: orientation
def read():
    global robotsInfo, ballsInfo
    # robotsInfo[:, :, 0] = np.nan
    while (ssl_wrapper.ParseFromString(ssl_fifo.read()) == 0):
        pass
    for robot in ssl_wrapper.detection.robots_blue:
        robotsInfo[0, robot.robot_id, 0] = robot.x
        robotsInfo[0, robot.robot_id, 1] = robot.y
        robotsInfo[0, robot.robot_id, 2] = robot.orientation
    for robot in ssl_wrapper.detection.robots_yellow:
        robotsInfo[1, robot.robot_id, 0] = robot.x
        robotsInfo[1, robot.robot_id, 1] = robot.y
        robotsInfo[1, robot.robot_id, 2] = robot.orientation
    ballsInfo = np.empty([len(ssl_wrapper.detection.balls), 2])
    for idx, ball in enumerate(ssl_wrapper.detection.balls):
        ballsInfo[idx, 0] = ball.x
        ballsInfo[idx, 1] = ball.y


def getGeometry():
    ssl_geometry = ssl_wrapper.geometry
    ssl_field = ssl_geometry.field
    return ssl_field.field_length, ssl_field.field_width


def close():
    ssl_fifo.close()