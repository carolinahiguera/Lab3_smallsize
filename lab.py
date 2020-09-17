import os
import signal
import subprocess
import time
from movement_functions import *
import cam_reader as cam
import sim_sender as rtx
import numpy as np

# PARAMETERS
# Default External Parameters
SIM_FLAG = True    # True=simulation, False=Real
NUM_CAMERAS = 4
DRAW_SCALE = 0.10

MAX_ROBOTS = 1
TEAM = 0 # Blue=0,  Yellow=1


# Robot movements
robot_move_abs = np.zeros([2, MAX_ROBOTS]) # Absolute to court
robot_move_rel = np.zeros([3, MAX_ROBOTS]) # Absolute to robot
# Ball
ball = (0, 0)
ball_ant = (0, 0)


pkgManager = subprocess.Popen(["pkgManager/pkgManager", str(NUM_CAMERAS)])
time.sleep(0.3)

# Initialization
rtx.init()
cam.init(MAX_ROBOTS)


alphasDeg = [45, 45+90, 45+2*90, 45+3*90]
# alphasDeg = [40, 180-40, 180+40, 360-40]
# alphasDeg = [50, 180-50, 180+50, 360-50]
alphas = np.radians(alphasDeg)
# D matrix
Dmat = np.array( [ [-np.sin(alphas[0]), np.cos(alphas[0]), 1],
                   [-np.sin(alphas[1]), np.cos(alphas[1]), 1],
                   [-np.sin(alphas[2]), np.cos(alphas[2]), 1],
                   [-np.sin(alphas[3]), np.cos(alphas[3]), 1] ] )


# Receiving geometry data
print('Waiting for geometry data...')
done = False
while not done:
    print('Waiting for geometry data...')
    cam.read()
    done = cam.ssl_wrapper.HasField('geometry')
# print('Geometry data received.')
diagCourt = math.sqrt(cam.getGeometry()[0]**2+cam.getGeometry()[1]**2)


def ballFiltering():
    global ball, ball_ant
    ball = ball_ant
    if len(cam.ballsInfo) > 0:
        dist = 10e6
        for id_ball in range(0, len(cam.ballsInfo)):
            ball_tmp = (cam.ballsInfo[id_ball, 0], cam.ballsInfo[id_ball, 1])
            if dist > np.sqrt((ball_ant[0] - ball_tmp[0]) ** 2 + (ball_ant[1] - ball_tmp[1]) ** 2):
                ball = ball_tmp
                dist = np.sqrt((ball_ant[0] - ball_tmp[0]) ** 2 + (ball_ant[1] - ball_tmp[1]) ** 2)
        ball_ant = ball

robot_move_abs[0] = 1.0
robot_move_abs[1] = 0.0

for i in range(500):
    # Camera reading
    cam.read()
    robot_move_rel[0:2] = rotateAxisMat(robot_move_abs[0], robot_move_abs[1], cam.robotsInfo[TEAM, :, 2])
    # BALL FILTERING
    ballFiltering()


    # motVel = np.matmul(Dmat, [robot_move_rel[0], robot_move_rel[1], robot_move_rel[2] ] )
    # motVel[0] = motVel[0] if motVel[0] <= 1.0 else 1.0
    # motVel[1] = motVel[1] if motVel[1] <= 1.0 else 1.0
    # motVel[2] = motVel[2] if motVel[2] <= 1.0 else 1.0
    # motVel[3] = motVel[3] if motVel[3] <= 1.0 else 1.0

    # # Create movement package
    # rtx.package_init(MAX_ROBOTS)
    # rtx.w0 = motVel[0] 
    # rtx.w1 = motVel[1] 
    # rtx.w2 = motVel[2] 
    # rtx.w3 = motVel[3] 
    # rtx.send_wheelsVel()

    # Create movement package
    rtx.package_init(MAX_ROBOTS)
    rtx.vx = robot_move_rel[0]
    rtx.vy = robot_move_rel[1]
    rtx.vw = robot_move_rel[2]
    rtx.send()

robot_move_rel[0] = 0.0
robot_move_rel[1] = 0.0
robot_move_rel[2] = 0.0
rtx.package_init(MAX_ROBOTS)
rtx.vx = robot_move_rel[0]
rtx.vy = robot_move_rel[1]
rtx.vw = robot_move_rel[2]
rtx.send()

# cam.close()
# os.killpg(os.getpgid(pkgManager.pid), signal.SIGTERM)


