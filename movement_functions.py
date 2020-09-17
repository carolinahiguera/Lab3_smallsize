import math
import numpy as np

# Performs geometric transportation
def rotateAxis(x, y, alpha):
    theta = - math.pi / 2 - alpha
    X = x * math.cos(theta) - y * math.sin(theta)
    Y = -x * math.sin(theta) - y * math.cos(theta)
    return X, Y


def rotateAxisMat(x, y, alpha):
    theta = - math.pi / 2 - alpha
    X = x * np.cos(theta) - y * np.sin(theta)
    Y = - x * np.sin(theta) - y * np.cos(theta)
    return np.array([X, Y])


def pid(m, theta, P, notStop):
    ctrl = (m/10817)*P
    if notStop:
        ctrl += 0.5 #0.5
    xnew = ctrl * math.cos(theta)
    ynew = ctrl * math.sin(theta)
    m = math.sqrt((xnew)**2+(ynew)**2)
    if m > 1:
        xnew = math.cos(theta)
        ynew = math.sin(theta)
    return xnew, ynew


def calcLinearControl(x1, y1, x2, y2, P, notStop=False):
    m = math.sqrt((x1-x2)**2+(y1-y2)**2)
    dy = y2-y1
    dx = x2-x1
    theta = math.atan2(dy, dx)
    # grad = theta*180/math.pi
    return pid(m, theta, P, notStop) + (m, )


def calcAngularControl( xini, yini, xdest, ydest, cx, cy, Pcp, Pw, cw, notStop=False):
    #PID radio
    target_R = math.sqrt((cx-xdest)**2+(cy-ydest)**2)
    R = math.sqrt((cx-xini)**2+(cy-yini)**2)
    errR = target_R - R
    Vcp = errR/300.0 * Pcp
    #PID angulo
    target_angle = math.atan2(ydest-cy, xdest-cx)
    angle = math.atan2(yini-cy, xini-cx)
    if cw:
        if target_angle > angle:
            angle = angle + 2*math.pi
        err_angle = angle - target_angle
    else:
        if target_angle < angle:
            angle = angle - 2*math.pi
        err_angle = angle - target_angle
    # if cw=1, err_angle>0
    # if xw=0, err_angle<0
    Vw = (err_angle/(2*math.pi)) * Pw
    if notStop:
        if cw:
            Vw += 0.5
        else:
            Vw -= 0.5
    theta = math.atan2(Vcp, Vw)
    m = math.sqrt((Vw) ** 2 + (Vcp) ** 2)
    if m > 1:
        Vw = math.cos(theta)
        Vcp = math.sin(theta)
    vx_, vy_ = rotateAxis(Vw, Vcp, -angle)
    return vx_, -vy_, errR, err_angle

