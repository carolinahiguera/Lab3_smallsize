import os
import signal
import subprocess
import time
from movement_functions import *
import cam_reader as cam
import sim_sender as rtx
import numpy as np

# PARAMETERS
SIM_FLAG = True    #Dejar siempre en True
NUM_CAMERAS = 4	   #En el simulador se tienen 4 camaras

MAX_ROBOTS = 1     #cantidad de robots por equipo
TEAM = 0 # Blue=0,  Yellow=1    #dejar siempre 0


# Robot movements
robot_move_abs = np.zeros([2, MAX_ROBOTS]) # Vx y Vy en marco global
robot_move_rel = np.zeros([3, MAX_ROBOTS]) # Vx, Vy y theta en marco local del robot

# Ball
ball = (0, 0)  #coordenada actual (x,y) en marco global de la bola
ball_ant = (0, 0) #coordenada anterior (x,y) en marco global de la bola


# ------------------ NO MODIFICAR ------------------------------
#lanzar proceso para comunicarse con grSim
pkgManager = subprocess.Popen(["pkgManager/pkgManager", str(NUM_CAMERAS)])
time.sleep(0.3)

# inicializar comunicacion
rtx.init()
# inicializar el objeto que el que se recibe info de las camaras
cam.init(MAX_ROBOTS)

print('Waiting for geometry data from grSim...')
done = False
while not done:
    print('Waiting for geometry data from grSim...')
    cam.read()
    done = cam.ssl_wrapper.HasField('geometry')
print('Done')

# Filtro de Kalman para la bola
# actualiza la posicion de la bola en el marco global
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
# ------------------------------------

# ------------- TU CODIGO AQUI -------------------------

# establecer un perfil de velocidad en marco global
robot_move_abs[0] = 1.0
robot_move_abs[1] = 0.0

# aplicar el perfil por n steps
for i in range(500):
    # leer las camaras
    cam.read()
    # convertir velocidad en marco global a marco local
    robot_move_rel[0:2] = rotateAxisMat(robot_move_abs[0], robot_move_abs[1], cam.robotsInfo[TEAM, :, 2])
    # actualizar posicion de la bola
    ballFiltering()

    # crear paquete de velocidad del robot (en marco local) para enviar al simulador
    rtx.package_init(MAX_ROBOTS)
    rtx.vx = robot_move_rel[0] #vel lineal en x
    rtx.vy = robot_move_rel[1] #vel lineal en y
    rtx.vw = robot_move_rel[2] #vel angular
    # enviar el paquete
    rtx.send()

# detener el robot
robot_move_rel[0] = 0.0
robot_move_rel[1] = 0.0
robot_move_rel[2] = 0.0
rtx.package_init(MAX_ROBOTS)
rtx.vx = robot_move_rel[0]
rtx.vy = robot_move_rel[1]
rtx.vw = robot_move_rel[2]
rtx.send()

# ------------- HASTA AQUI ----------------------

#  NO MODIFICAR
cam.close()
os.killpg(os.getpgid(pkgManager.pid), signal.SIGTERM)
#--------------------


