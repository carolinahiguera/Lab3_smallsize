import sys
sys.path.insert(0, './sender_sim')
from sender import SSL_Sender

ssl_sender = None
vx = []
vy = []
vw = []
kx = []
kz = []
sp = []


def init():
    global ssl_sender
    global x
    ssl_sender = SSL_Sender()
    x = 1


def package_init(num_robots):
    global vx, vy, vw, kx, kz, sp
    vx = [0.0] * num_robots
    vy = [0.0] * num_robots
    vw = [0.0] * num_robots
    kx = [0.0] * num_robots
    kz = [0.0] * num_robots
    sp = [0.0] * num_robots
    w0 = [0.0] * num_robots
    w1 = [0.0] * num_robots
    w2 = [0.0] * num_robots
    w3 = [0.0] * num_robots


def send():
    global vx, vy, vw
    vx = (vx * 0.8 * 0.4) * 3.24
    vy = (vy * 0.8 * 0.4) * 3.24
    vw = (vw * 0.2 * 0.4) * 38.74
    ssl_sender.sendVels(0, vx, vy, vw, kx, kz, sp)

def send_wheelsVel():
    global w0, w1, w2, w3    
    ssl_sender.sendWheels(0, w0, w1, w2, w3, kx, kz, sp)

