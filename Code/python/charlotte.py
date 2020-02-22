from __future__ import division
import numpy as np
import time
# import Adafruit_PCA9685
from cvxopt import matrix, solvers
# Imports for plot
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# Imports for COM
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

# Commonly used functions
cos = np.cos
sin = np.sin
pi = np.pi
atan2 = np.arctan2
dot = np.dot
array = np.array
ar = np.arange


def main():
    q = initialize()

    # Forward kinematics [leg1,leg2,leg3,leg4]:
    leg1_pos = fkinematics([q[3], q[4], q[5]], 1, 1)
    leg2_pos = fkinematics([q[6], q[7], q[8]], 2, 1)
    leg3_pos = fkinematics([q[9], q[10], q[11]], 3, 1)
    leg4_pos = fkinematics([q[12], q[13], q[14]], 4, 1)

    # Desired position [posb,leg1,leg2,leg3,leg4,rotb]:
    xd = np.zeros(18)
    xd[0:3] = [0, 1, 0]
    xd[3:6] = array([3, 2, 0]) + leg1_pos
    xd[6:9] = array([1, 4, 0]) + leg2_pos
    xd[9:12] = array([2, 2, 0]) + leg3_pos
    xd[12:15] = array([0, 1, 0]) + leg4_pos
    xd[15:18] = np.radians([10, 0, 0])

    # Maximum number of iterations
    max_iter = 40
    # Time between signals
    d_t = 0.01
    # Gain of quadratic function
    lamb = -10
    # Maximum tolerance for minimization
    tol = 0.001
    # Weights: [posb,leg1,leg2,leg3,leg4,rotb]
    w = array([1, 1, 1, 1, 1, 1])
    # Quadratic program
    [qf, i] = fcomp(q, xd, max_iter, d_t, lamb, w, tol)
    # Move servos to desired position
    # move_servo(qf, i, 3)
    # Compare desired and current positions (x,y,z)
    compare(qf, xd, i)
    # Find the center of mass
    com = cmass(q)
    # Simulate the body configuration
    simulation(qf, com, i)


def initialize():
    # q: Initial configuration vector [posb,leg1,leg2,leg3,leg4,rotb]
    q = np.zeros(18)
    q[0:3] = [0, 0, 0]
    q[3:6] = np.radians([90, 90, 90])
    q[6:9] = np.radians([90, 90, 90])
    q[9:12] = np.radians([90, 90, 90])
    q[12:15] = np.radians([90, 90, 90])
    q[15:18] = np.radians([0, 0, 0])
    # move_servo(q, 1, 1)
    return q


def cmass(q):
    # Determines if the center of mass (COM) is in a stable configuration
    [com, feet] = com_system(q)
    # Projects the COM in the ground
    point = Point(com[0], com[1])
    # Finds if the point is inside the polygon
    polygon = Polygon([feet[ar(2)], feet[ar(2) + 2], feet[ar(2) + 6]])
    print('Is the point inside the polygon?: ', polygon.contains(point))
    return com


def simulation(qf, com, i):
    # Simulate the current configuration of the system
    plt.ion()
    fig = plt.figure()

    for j in range(i):
        plt.clf()
        # Manually set the limits
        ax = fig.gca(projection='3d')
        ax.set_xlim(-40, 40)
        ax.set_ylim(-40, 40)
        ax.set_zlim(-40, 20)

        # Manually label the axes
        ax.set_xlabel('X-axis (cm)')
        ax.set_ylabel('Y-axis (cm)')
        ax.set_zlabel('Z-axis (cm)')

        # Finds and locates the points and vectors for the figure
        body_simulation(qf[j], ax, com)
        fig.canvas.flush_events()
        time.sleep(0.001)

    plt.ioff()
    plt.show()


def move_servo(q, i, itr):
    # Move the servos to the desired configuration
    pwm = Adafruit_PCA9685.PCA9685()  # Start PWM
    pwm.set_pwm_freq(60)  # Set frequency to 60hz, good for servos.

    # Configure min and max servo pulse lengths
    servo_min = 190  # Min pulse length out of 4096
    servo_max = 690  # Max pulse length out of 4096

    def valmap(value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

    def set_servo(channel, angle):
        duty = int(valmap(angle, 0, 170, servo_min, servo_max))
        pwm.set_pwm(channel, 0, duty)

    sp = np.around(np.linspace(0, i, itr))

    for j in range(sp.size):
        # Leg
        set_servo(1, q[sp[j]][3] * 180 / pi)
        set_servo(5, q[sp[j]][6] * 180 / pi)
        set_servo(9, q[sp[j]][9] * 180 / pi)
        set_servo(13, q[sp[j]][12] * 180 / pi)
        # Knee
        set_servo(2, q[sp[j]][4] * 180 / pi)
        set_servo(6, q[sp[j]][7] * 180 / pi)
        set_servo(10, q[sp[j]][10] * 180 / pi)
        set_servo(14, q[sp[j]][13] * 180 / pi)
        # Foot
        set_servo(3, q[sp[j]][5] * 180 / pi)
        set_servo(7, q[sp[j]][8] * 180 / pi)
        set_servo(11, q[sp[j]][11] * 180 / pi)
        set_servo(15, q[sp[j]][14] * 180 / pi)
        time.sleep(.01)


def compare(qf, xd, i):
    # Final position
    posb = [qf[i - 1][0], qf[i - 1][1], qf[i - 1][2]]
    rot = [qf[i - 1][15], qf[i - 1][16], qf[i - 1][17]]
    [leg1f, __] = ftransform([qf[i - 1][3], qf[i - 1][4], qf[i - 1][5]], posb, rot, 1)
    [leg2f, __] = ftransform([qf[i - 1][6], qf[i - 1][7], qf[i - 1][8]], posb, rot, 2)
    [leg3f, __] = ftransform([qf[i - 1][9], qf[i - 1][10], qf[i - 1][11]], posb, rot, 3)
    [leg4f, __] = ftransform([qf[i - 1][12], qf[i - 1][13], qf[i - 1][14]], posb, rot, 4)

    print("Desired pos:", xd[ar(3) + 3], "\n", "           ", xd[ar(3) + 6])
    print("            ", xd[ar(3) + 9], "\n", "           ", xd[ar(3) + 12])
    print("Final pos  :", np.around(leg1f, 8), "\n", "           ", np.around(leg2f, 8))
    print("            ", np.around(leg3f, 8), "\n", "           ", np.around(leg4f, 8))
    print("Body pos   :", xd[ar(3)], qf[i - 1][ar(3)])
    print("Body rot   :", xd[ar(3) + 15], qf[i - 1][ar(3) + 15])


def q2rot(a, b, c):
    rx = array([[1, 0, 0],
                [0, cos(a), -sin(a)],
                [0, sin(a), cos(a)]])

    ry = array([[cos(b), 0, sin(b)],
                [0, 1, 0],
                [-sin(b), 0, cos(b)]])

    rz = array([[cos(c), -sin(c), 0],
                [sin(c), cos(c), 0],
                [0, 0, 1]])

    rotm = rz.dot(ry).dot(rx)
    return rotm


def rot2q(rotm):
    c = atan2(rotm[1][0], rotm[0][0])
    b = atan2(-rotm[2][0], np.sqrt((rotm[2][1] ** 2) + (rotm[2][2] ** 2)))
    a = atan2(rotm[2][1], rotm[2][2])

    return [a, b, c]


def fkinematics(q, leg, mode):
    # This function finds the forward kinematics of each leg of the robot.
    global leg_end, leg_base, leg_srt, leg_mid, pos
    r1 = 5.5     # Distance from servo 1 to 2
    r2 = 7.5     # Distance from servo 2 to 3
    r3 = 22.5    # Distance from servo 3 to effector
    r4 = 10.253  # Distance from base to servo 1
    leg1_rot = -135 * (pi / 180)  # Position of servo 1 with respect to base
    leg2_rot = -45 * (pi / 180)   # Position of servo 2 with respect to base
    leg3_rot = 135 * (pi / 180)   # Position of servo 3 with respect to base
    leg4_rot = 45 * (pi / 180)    # Position of servo 4 with respect to base

    # Denavit-Hartenberg matrices
    m_1 = array([[cos(q[0]), 0, sin(q[0]), r1 * cos(q[0])],
                 [sin(q[0]), 0, -cos(q[0]), r1 * sin(q[0])],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])

    m_2 = array([[cos(q[1] + pi / 2), sin(q[1] + pi / 2), 0, -r2 * cos(q[1] + pi / 2)],
                 [sin(q[1] + pi / 2), -cos(q[1] + pi / 2), 0, -r2 * sin(q[1] + pi / 2)],
                 [0, 0, -1, 0],
                 [0, 0, 0, 1]])

    m_3 = array([[cos(q[2]), sin(q[2]), 0, -r3 * cos(q[2])],
                 [sin(q[2]), -cos(q[2]), 0, -r3 * sin(q[2])],
                 [0, 0, -1, 0],
                 [0, 0, 0, 1]])

    # Position of the legs with respect to the base
    leg1 = array([[cos(leg1_rot), -sin(leg1_rot), 0, r4],
                  [sin(leg1_rot), cos(leg1_rot), 0, -r4],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    leg2 = array([[cos(leg2_rot), -sin(leg2_rot), 0, r4],
                  [sin(leg2_rot), cos(leg2_rot), 0, r4],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    leg3 = array([[cos(leg3_rot), -sin(leg3_rot), 0, -r4],
                  [sin(leg3_rot), cos(leg3_rot), 0, -r4],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    leg4 = array([[cos(leg4_rot), -sin(leg4_rot), 0, -r4],
                  [sin(leg4_rot), cos(leg4_rot), 0, r4],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    # Homogeneous Transformation Matrix
    if mode == 1:
        if leg == 1:
            leg_end = leg1.dot(m_1).dot(m_2).dot(m_3)
        elif leg == 2:
            leg_end = leg2.dot(m_1).dot(m_2).dot(m_3)
        elif leg == 3:
            leg_end = leg3.dot(m_1).dot(m_2).dot(m_3)
        elif leg == 4:
            leg_end = leg4.dot(m_1).dot(m_2).dot(m_3)

        # Position vector
        pos = array([0.0, 0.0, 0.0])
        pos[0] = leg_end[0][3]  # End-effector position on the x-axis
        pos[1] = leg_end[1][3]  # End-effector position on the y-axis
        pos[2] = leg_end[2][3]  # End-effector position on the z-axis

    elif mode == 2:
        if leg == 1:
            leg_base = leg1
            leg_srt = leg1.dot(m_1)
            leg_mid = leg1.dot(m_1).dot(m_2)
            leg_end = leg1.dot(m_1).dot(m_2).dot(m_3)
        elif leg == 2:
            leg_base = leg2
            leg_srt = leg2.dot(m_1)
            leg_mid = leg2.dot(m_1).dot(m_2)
            leg_end = leg2.dot(m_1).dot(m_2).dot(m_3)
        elif leg == 3:
            leg_base = leg3
            leg_srt = leg3.dot(m_1)
            leg_mid = leg3.dot(m_1).dot(m_2)
            leg_end = leg3.dot(m_1).dot(m_2).dot(m_3)
        elif leg == 4:
            leg_base = leg4
            leg_srt = leg4.dot(m_1)
            leg_mid = leg4.dot(m_1).dot(m_2)
            leg_end = leg4.dot(m_1).dot(m_2).dot(m_3)

        # Position vector [base, leg_start, leg_middle, leg_end]: (x,y,z)
        p0 = [leg_base[0][3], leg_base[1][3], leg_base[2][3]]
        p1 = [leg_srt[0][3], leg_srt[1][3], leg_srt[2][3]]
        p2 = [leg_mid[0][3], leg_mid[1][3], leg_mid[2][3]]
        p3 = [leg_end[0][3], leg_end[1][3], leg_end[2][3]]
        pos = np.concatenate((p0, p1, p2, p3), axis=0)

    return pos


def ftransform(ang, posb, rot, op):
    # This function returns the position of the end-effector according to the position and orientation of
    # the base. This is possible by calculating the jacobian of each leg a locating it in a homogeneous
    # transformation matrix. This function returns the jacobian and position of the selected leg.
    global jacobian, mf
    q1 = ang[0]
    q2 = ang[1]
    q3 = ang[2]

    # Forward kinematics and jacobian of each leg
    if op == 1:
        mf = array([[sin(q2 - q3) * cos(q1 + pi / 4), cos(q2 - q3) * cos(q1 + pi / 4), -sin(q1 + pi / 4),
                     (11 * 2 ** (1 / 2) * sin(q1)) / 4 - (11 * 2 ** (1 / 2) * cos(q1)) / 4 - (
                             15 * cos(q1 + pi / 4) * sin(q2)) / 2 + (
                             45 * cos(q2) * cos(q1 + pi / 4) * sin(q3)) / 2 - (
                             45 * cos(q3) * cos(q1 + pi / 4) * sin(q2)) / 2 + 10253 / 1000],
                    [sin(q2 - q3) * sin(q1 + pi / 4), cos(q2 - q3) * sin(q1 + pi / 4), cos(q1 + pi / 4),
                     (45 * cos(q2) * sin(q3) * sin(q1 + pi / 4)) / 2 - (11 * 2 ** (1 / 2) * cos(q1)) / 4 - (
                             11 * 2 ** (1 / 2) * sin(q1)) / 4 - (15 * sin(q2) * sin(q1 + pi / 4)) / 2 - (
                             45 * cos(q3) * sin(q2) * sin(q1 + pi / 4)) / 2 - 10253 / 1000],
                    [cos(q2 - q3), -sin(q2 - q3), 0, - (45 * cos(q2 - q3)) / 2 - (15 * cos(q2)) / 2],
                    [0, 0, 0, 1]])

        jacobian = array([[(sin(q1 + pi / 4) * (45 * sin(q2 - q3) + 15 * sin(q2) + 11)) / 2,
                           -(15 * cos(q1 + pi / 4) * (3 * cos(q2 - q3) + cos(q2))) / 2,
                           (45 * cos(q2 - q3) * cos(q1 + pi / 4)) / 2],
                          [-(cos(q1 + pi / 4) * (45 * sin(q2 - q3) + 15 * sin(q2) + 11)) / 2,
                           -(15 * sin(q1 + pi / 4) * (3 * cos(q2 - q3) + cos(q2))) / 2,
                           (45 * cos(q2 - q3) * sin(q1 + pi / 4)) / 2],
                          [0, (45 * sin(q2 - q3)) / 2 + (15 * sin(q2)) / 2, -(45 * sin(q2 - q3)) / 2]])
    elif op == 2:
        mf = array([[-sin(q2 - q3) * sin(q1 + pi / 4), -cos(q2 - q3) * sin(q1 + pi / 4), -cos(q1 + pi / 4),
                     (15 * sin(q2) * sin(q1 + pi / 4)) / 2 + (11 * 2 ** (1 / 2) * cos(q1)) / 4 + (
                             11 * 2 ** (1 / 2) * sin(q1)) / 4 - (45 * cos(q2) * sin(q3) * sin(q1 + pi / 4)) / 2 + (
                             45 * cos(q3) * sin(q2) * sin(q1 + pi / 4)) / 2 + 10253 / 1000],
                    [sin(q2 - q3) * cos(q1 + pi / 4), cos(q2 - q3) * cos(q1 + pi / 4), -sin(q1 + pi / 4),
                     (11 * 2 ** (1 / 2) * sin(q1)) / 4 - (11 * 2 ** (1 / 2) * cos(q1)) / 4 - (
                             15 * cos(q1 + pi / 4) * sin(q2)) / 2 + (
                             45 * cos(q2) * cos(q1 + pi / 4) * sin(q3)) / 2 - (
                             45 * cos(q3) * cos(q1 + pi / 4) * sin(q2)) / 2 + 10253 / 1000],
                    [cos(q2 - q3), -sin(q2 - q3), 0, - (45 * cos(q2 - q3)) / 2 - (15 * cos(q2)) / 2],
                    [0, 0, 0, 1]])

        jacobian = array([[(cos(q1 + pi / 4) * (45 * sin(q2 - q3) + 15 * sin(q2) + 11)) / 2,
                           (15 * sin(q1 + pi / 4) * (3 * cos(q2 - q3) + cos(q2))) / 2,
                           -(45 * cos(q2 - q3) * sin(q1 + pi / 4)) / 2],
                          [(sin(q1 + pi / 4) * (45 * sin(q2 - q3) + 15 * sin(q2) + 11)) / 2,
                           -(15 * cos(q1 + pi / 4) * (3 * cos(q2 - q3) + cos(q2))) / 2,
                           (45 * cos(q2 - q3) * cos(q1 + pi / 4)) / 2],
                          [0, (45 * sin(q2 - q3)) / 2 + (15 * sin(q2)) / 2, -(45 * sin(q2 - q3)) / 2]])
    elif op == 3:
        mf = array([[sin(q2 - q3) * sin(q1 + pi / 4), cos(q2 - q3) * sin(q1 + pi / 4), cos(q1 + pi / 4),
                     (45 * cos(q2) * sin(q3) * sin(q1 + pi / 4)) / 2 - (11 * 2 ** (1 / 2) * cos(q1)) / 4 - (
                             11 * 2 ** (1 / 2) * sin(q1)) / 4 - (15 * sin(q2) * sin(q1 + pi / 4)) / 2 - (
                             45 * cos(q3) * sin(q2) * sin(q1 + pi / 4)) / 2 - 10253 / 1000],
                    [-sin(q2 - q3) * cos(q1 + pi / 4), -cos(q2 - q3) * cos(q1 + pi / 4), sin(q1 + pi / 4),
                     (15 * cos(q1 + pi / 4) * sin(q2)) / 2 + (11 * 2 ** (1 / 2) * cos(q1)) / 4 - (
                             11 * 2 ** (1 / 2) * sin(q1)) / 4 - (45 * cos(q2) * cos(q1 + pi / 4) * sin(q3)) / 2 + (
                             45 * cos(q3) * cos(q1 + pi / 4) * sin(q2)) / 2 - 10253 / 1000],
                    [cos(q2 - q3), -sin(q2 - q3), 0, - (45 * cos(q2 - q3)) / 2 - (15 * cos(q2)) / 2],
                    [0, 0, 0, 1]])

        jacobian = array([[-(cos(q1 + pi / 4) * (45 * sin(q2 - q3) + 15 * sin(q2) + 11)) / 2,
                           -(15 * sin(q1 + pi / 4) * (3 * cos(q2 - q3) + cos(q2))) / 2,
                           (45 * cos(q2 - q3) * sin(q1 + pi / 4)) / 2],
                          [-(sin(q1 + pi / 4) * (45 * sin(q2 - q3) + 15 * sin(q2) + 11)) / 2,
                           (15 * cos(q1 + pi / 4) * (3 * cos(q2 - q3) + cos(q2))) / 2,
                           -(45 * cos(q2 - q3) * cos(q1 + pi / 4)) / 2],
                          [0, (45 * sin(q2 - q3)) / 2 + (15 * sin(q2)) / 2, -(45 * sin(q2 - q3)) / 2]])
    elif op == 4:
        mf = array([[-sin(q2 - q3) * cos(q1 + pi / 4), -cos(q2 - q3) * cos(q1 + pi / 4), sin(q1 + pi / 4),
                     (15 * cos(q1 + pi / 4) * sin(q2)) / 2 + (11 * 2 ** (1 / 2) * cos(q1)) / 4 - (
                             11 * 2 ** (1 / 2) * sin(q1)) / 4 - (45 * cos(q2) * cos(q1 + pi / 4) * sin(q3)) / 2 + (
                             45 * cos(q3) * cos(q1 + pi / 4) * sin(q2)) / 2 - 10253 / 1000],
                    [-sin(q2 - q3) * sin(q1 + pi / 4), -cos(q2 - q3) * sin(q1 + pi / 4), -cos(q1 + pi / 4),
                     (15 * sin(q2) * sin(q1 + pi / 4)) / 2 + (11 * 2 ** (1 / 2) * cos(q1)) / 4 + (
                             11 * 2 ** (1 / 2) * sin(q1)) / 4 - (45 * cos(q2) * sin(q3) * sin(q1 + pi / 4)) / 2 + (
                             45 * cos(q3) * sin(q2) * sin(q1 + pi / 4)) / 2 + 10253 / 1000],
                    [cos(q2 - q3), -sin(q2 - q3), 0, - (45 * cos(q2 - q3)) / 2 - (15 * cos(q2)) / 2],
                    [0, 0, 0, 1]])

        jacobian = array([[-(sin(q1 + pi / 4) * (45 * sin(q2 - q3) + 15 * sin(q2) + 11)) / 2,
                           (15 * cos(q1 + pi / 4) * (3 * cos(q2 - q3) + cos(q2))) / 2,
                           -(45 * cos(q2 - q3) * cos(q1 + pi / 4)) / 2],
                          [(cos(q1 + pi / 4) * (45 * sin(q2 - q3) + 15 * sin(q2) + 11)) / 2,
                           (15 * sin(q1 + pi / 4) * (3 * cos(q2 - q3) + cos(q2))) / 2,
                           -(45 * cos(q2 - q3) * sin(q1 + pi / 4)) / 2],
                          [0, (45 * sin(q2 - q3)) / 2 + (15 * sin(q2)) / 2, -(45 * sin(q2 - q3)) / 2]])

    # Homogeneous Transformation Matrix
    rotm = q2rot(rot[0], rot[1], rot[2])

    trans = array([[rotm[0][0], rotm[0][1], rotm[0][2], posb[0]],
                   [rotm[1][0], rotm[1][1], rotm[1][2], posb[1]],
                   [rotm[2][0], rotm[2][1], rotm[2][2], posb[2]],
                   [0, 0, 0, 1]])

    mf = dot(trans, mf)
    jacobian = dot(rotm, jacobian)
    position = [mf[0][3], mf[1][3], mf[2][3]]

    return [position, jacobian]


def fcomp(q, xd, max_iter, d_t, lamb, w, tol):
    # This function manages the minimization program and find the error of the desired function
    i = 0
    qf = np.zeros((max_iter, 18))

    while i < max_iter:
        # Quadratic programming
        [h, f] = costfunc(q, xd, lamb, w)
        h = matrix((h + h.T) / 2)
        f = matrix(f)
        # Limits of the quadratic function
        # G = matrix(np.vstack([np.eye(18),-np.eye(18)]))
        # h = array(36*[1e5,])
        # h[ar(18)] = np.minimum(h[ar(18)],(pi - q)/dT)
        # h[ar(18)+18] = np.maximum(-1e-5, (0 - q)/dT)
        # h[ar(18)+18] = -1e-5
        # h = matrix(h)
        # Solver configuration
        solvers.options['show_progress'] = False
        solvers.options['maxiters'] = 1000
        solvers.options['abstol'] = 1e-12
        solvers.options['reltol'] = 1e-12
        solvers.options['feastol'] = 1e-100
        solvers.options['xtol'] = 1e-12
        sol = solvers.qp(h, f)
        dq = sol['x']
        dq = array(dq)

        # Update the position vector
        q[ar(15)] = q[ar(15)] + d_t * dq[ar(15)].T

        # Update the orientation vector
        rot_axis = dot(array([[cos(q[16]) * cos(q[17]), -sin(q[17]), 0],
                              [cos(q[16]) * sin(q[17]), cos(q[17]), 0],
                              [-sin(q[16]), 0, 1]]), [dq[15], dq[16], dq[17]])

        skw = array([[0, -rot_axis[2], rot_axis[1]],
                     [rot_axis[2], 0, -rot_axis[0]],
                     [-rot_axis[1], rot_axis[0], 0]])

        rgs = np.eye(3) + sin(d_t) * skw + (1 - cos(d_t)) * skw ** 2
        rot = q2rot(q[15], q[16], q[17]).dot(rgs)
        [q[15], q[16], q[17]] = rot2q(rot)
        qf[i] = q
        err = calc_err(q, xd)
        if err <= tol:
            i = i + 1
            break
        i = i + 1

    return [qf, i]

    # for j in range(0,max_iter):
    #    body_simulation(qf[j])
    #    time.sleep(0.00000000001)


def calc_err(q, xd):
    leg1f = fkinematics([q[3], q[4], q[5]], 1, 1)
    leg2f = fkinematics([q[6], q[7], q[8]], 2, 1)
    leg3f = fkinematics([q[9], q[10], q[11]], 3, 1)
    leg4f = fkinematics([q[12], q[13], q[14]], 4, 1)
    err_leg1 = xd[ar(3) + 3] - leg1f
    err_leg2 = xd[ar(3) + 6] - leg2f
    err_leg3 = xd[ar(3) + 9] - leg3f
    err_leg4 = xd[ar(3) + 12] - leg4f
    err_posb = xd[ar(3)] - q[ar(3)]
    err_rotb = xd[ar(3) + 15] - q[ar(3) + 15]
    err = np.sqrt(
        err_leg1[0] ** 2 + err_leg1[1] ** 2 + err_leg1[2] ** 2 + err_leg2[0] ** 2 + err_leg2[1] ** 2 + err_leg2[
            2] ** 2 + err_leg3[0] ** 2 +
        err_leg3[1] ** 2 + err_leg3[2] ** 2 + err_leg4[0] ** 2 + err_leg4[1] ** 2 + err_leg4[2] ** 2 + err_posb[
            0] ** 2 + err_posb[1] ** 2 +
        err_posb[2] ** 2 + err_rotb[0] ** 2 + err_rotb[1] ** 2 + err_rotb[2] ** 2)

    return err


def costfunc(q, xd, lamb, w):
    # This function finds the values of h and f in order to initialize the quadratic program.
    # The inputs are q: actuated and sub-actuated angles, xd: desired position vector, p: weights and lamb: gain.

    # Position and orientation of the base
    posb = q[0:3]
    rotb = q[15:18]

    # Leg 1:
    [pos1, ja1] = ftransform(q[3:6], posb, rotb, 1)
    # Distance vector of the end-effector with respect to base
    d1 = q[0:3] - pos1
    # Skew-Symmetric matrix
    sk1 = array([[0, -d1[2], d1[1]],
                 [d1[2], 0, -d1[0]],
                 [-d1[1], d1[0], 0]])
    j1 = np.concatenate((np.eye(3), ja1, np.zeros((3, 9)), sk1), axis=1)

    # Leg 2:
    [pos2, ja2] = ftransform(q[6:9], posb, rotb, 2)
    d2 = q[0:3] - pos2
    sk2 = array([[0, -d2[2], d2[1]],
                 [d2[2], 0, -d2[0]],
                 [-d2[1], d2[0], 0]])
    j2 = np.concatenate((np.eye(3), np.zeros((3, 3)), ja2, np.zeros((3, 6)), sk2), axis=1)

    # Leg 3:
    [pos3, ja3] = ftransform(q[9:12], posb, rotb, 3)
    d3 = q[0:3] - pos3
    sk3 = array([[0, -d3[2], d3[1]],
                 [d3[2], 0, -d3[0]],
                 [-d3[1], d3[0], 0]])
    j3 = np.concatenate((np.eye(3), np.zeros((3, 6)), ja3, np.zeros((3, 3)), sk3), axis=1)

    # Leg 4:
    [pos4, ja4] = ftransform(q[12:15], posb, rotb, 4)
    d4 = q[0:3] - pos4
    sk4 = array([[0, -d4[2], d4[1]],
                 [d4[2], 0, -d4[0]],
                 [-d4[1], d4[0], 0]])
    j4 = np.concatenate((np.eye(3), np.zeros((3, 9)), ja4, sk4), axis=1)

    # Jacobians of the base position and orientation
    j5 = np.concatenate((np.eye(3), np.zeros((3, 15))), axis=1)
    j6 = np.concatenate((np.zeros((3, 15)), np.eye(3)), axis=1)

    # Values of h and f (Hessian and vector of linear elements):
    h = w[0] * j1.T.dot(j1) + w[1] * j2.T.dot(j2) + w[2] * j3.T.dot(j3) + w[3] * j4.T.dot(j4) + w[4] * j5.T.dot(j5) + w[
        5] * j6.T.dot(j6)

    f = -2 * (w[1] * lamb * (pos1 - xd[3:6]).T.dot(j1) + w[2] * lamb * (pos2 - xd[6:9]).T.dot(j2) + w[3] * lamb * (
            pos3 - xd[9:12]).T.dot(j3) + w[4] * lamb * (pos4 - xd[12:15]).T.dot(j4) + w[0] * lamb * (
                      posb - xd[0:3]).T.dot(j5) + w[5] * lamb * (rotb - xd[15:18]).T.dot(j6))

    return [h, f]


def com_system(q):
    # Find the center of mass
    leg1_com = com_kinematics(q[ar(3) + 3], 1, q[ar(3)], q[ar(3) + 15])
    leg2_com = com_kinematics(q[ar(3) + 6], 2, q[ar(3)], q[ar(3) + 15])
    leg3_com = com_kinematics(q[ar(3) + 9], 3, q[ar(3)], q[ar(3) + 15])
    leg4_com = com_kinematics(q[ar(3) + 12], 4, q[ar(3)], q[ar(3) + 15])

    # COM of the base
    base = 0.7 * array([0, 0, 0])

    # COM position
    com = leg1_com[ar(3)] + leg1_com[ar(3) + 3] + leg1_com[ar(3) + 6] + leg2_com[ar(3)] + leg2_com[ar(3) + 3] + \
          leg2_com[ar(3) + 6] + leg3_com[ar(3)] + leg3_com[ar(3) + 3] + leg3_com[ar(3) + 6] + leg4_com[ar(3)] + \
          leg4_com[ar(3) + 3] + leg4_com[ar(3) + 6] + base

    # Location of the legs
    f1 = [leg1_com[9], leg1_com[10]]
    f2 = [leg2_com[9], leg2_com[10]]
    f3 = [leg3_com[9], leg3_com[10]]
    f4 = [leg4_com[9], leg4_com[10]]
    feet = np.concatenate((f1, f2, f3, f4), axis=0)

    return [com, feet]


def com_kinematics(q, leg, posb, rot):
    # Variables
    global mf, com3, com2, com1
    r1 = 5.5     # Distance from servo 1 to 2
    r2 = 7.5     # Distance from servo 2 to 3
    r2t = 3.75   # Distance from servo 2 to com2
    r3 = 22.5    # Distance from servo 3 to end-effector
    r3t = 11     # Distance from servo 3 to com3
    r4 = 10.253  # Distance from base to servo 1
    leg1_rot = -135 * (pi / 180)  # Position of servo 1 with respect to the base
    leg2_rot = -45 * (pi / 180)   # Position of servo 2 with respect to the base
    leg3_rot = 135 * (pi / 180)   # Position of servo 3 with respect to the base
    leg4_rot = 45 * (pi / 180)    # Position of servo 4 with respect to the base

    # Matrices
    m_1 = array([[cos(q[0]), 0, sin(q[0]), r1 * cos(q[0])],
                 [sin(q[0]), 0, -cos(q[0]), r1 * sin(q[0])],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])

    m_2 = array([[cos(q[1] + pi / 2), sin(q[1] + pi / 2), 0, -r2 * cos(q[1] + pi / 2)],
                 [sin(q[1] + pi / 2), -cos(q[1] + pi / 2), 0, -r2 * sin(q[1] + pi / 2)],
                 [0, 0, -1, 0],
                 [0, 0, 0, 1]])

    m_2t = array([[cos(q[1] + pi / 2), sin(q[1] + pi / 2), 0, -r2t * cos(q[1] + pi / 2)],
                  [sin(q[1] + pi / 2), -cos(q[1] + pi / 2), 0, -r2t * sin(q[1] + pi / 2)],
                  [0, 0, -1, 0],
                  [0, 0, 0, 1]])

    m_3 = array([[cos(q[2]), sin(q[2]), 0, -r3 * cos(q[2])],
                 [sin(q[2]), -cos(q[2]), 0, -r3 * sin(q[2])],
                 [0, 0, -1, 0],
                 [0, 0, 0, 1]])

    m_3t = array([[cos(q[2]), sin(q[2]), 0, -r3t * cos(q[2])],
                  [sin(q[2]), -cos(q[2]), 0, -r3t * sin(q[2])],
                  [0, 0, -1, 0],
                  [0, 0, 0, 1]])

    # Homogeneous Transformation Matrix
    if leg == 1:
        # Locate leg with respect to the base
        leg1 = array([[cos(leg1_rot), -sin(leg1_rot), 0, r4],
                      [sin(leg1_rot), cos(leg1_rot), 0, -r4],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        # Position and weight of the center of mass
        com1 = 0.075 * leg1
        com2 = 0.15 * dot(leg1, dot(m_1, m_2t))
        com3 = 0.2 * dot(leg1, dot(m_1, dot(m_2, m_3t)))
        mf = dot(leg1, dot(m_1, dot(m_2, m_3)))
    elif leg == 2:
        # Locate leg with respect to the base
        leg2 = array([[cos(leg2_rot), -sin(leg2_rot), 0, r4],
                      [sin(leg2_rot), cos(leg2_rot), 0, r4],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        # Position and weight of the center of mass
        com1 = 0.075 * leg2
        com2 = 0.15 * dot(leg2, dot(m_1, m_2t))
        com3 = 0.2 * dot(leg2, dot(m_1, dot(m_2, m_3t)))
        mf = dot(leg2, dot(m_1, dot(m_2, m_3)))
    elif leg == 3:
        # Locate leg with respect to the base
        leg3 = array([[cos(leg3_rot), -sin(leg3_rot), 0, -r4],
                      [sin(leg3_rot), cos(leg3_rot), 0, -r4],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        # Position and weight of the center of mass
        com1 = 0.075 * leg3
        com2 = 0.15 * dot(leg3, dot(m_1, m_2t))
        com3 = 0.2 * dot(leg3, dot(m_1, dot(m_2, m_3t)))
        mf = dot(leg3, dot(m_1, dot(m_2, m_3)))
    elif leg == 4:
        # Locate leg with respect to the base
        leg4 = array([[cos(leg4_rot), -sin(leg4_rot), 0, -r4],
                      [sin(leg4_rot), cos(leg4_rot), 0, r4],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        # Position and weight of the center of mass
        com1 = 0.075 * leg4
        com2 = 0.15 * dot(leg4, dot(m_1, m_2t))
        com3 = 0.2 * dot(leg4, dot(m_1, dot(m_2, m_3t)))
        mf = dot(leg4, dot(m_1, dot(m_2, m_3)))

    rotm = q2rot(rot[0], rot[1], rot[2])

    trans = array([[rotm[0][0], rotm[0][1], rotm[0][2], posb[0]],
                   [rotm[1][0], rotm[1][1], rotm[1][2], posb[1]],
                   [rotm[2][0], rotm[2][1], rotm[2][2], posb[2]],
                   [0, 0, 0, 1]])

    com1 = dot(trans, com1)
    com2 = dot(trans, com2)
    com3 = dot(trans, com3)
    mf = dot(trans, mf)

    # Position Vector
    p1 = [com1[0][3], com1[1][3], com1[2][3]]
    p2 = [com2[0][3], com2[1][3], com2[2][3]]
    p3 = [com3[0][3], com3[1][3], com3[2][3]]
    p4 = [mf[0][3], mf[1][3], mf[2][3]]
    position = np.concatenate((p1, p2, p3, p4), axis=0)

    return position


def body_simulation(q, ax, com):
    # Find the position of the legs
    l1_pos = fkinematics(q[ar(3) + 3], 1, 2)
    l2_pos = fkinematics(q[ar(3) + 6], 2, 2)
    l3_pos = fkinematics(q[ar(3) + 9], 3, 2)
    l4_pos = fkinematics(q[ar(3) + 12], 4, 2)

    # Homogeneous Transformation Matrix
    rotm = q2rot(q[15], q[16], q[17])
    trans = array([[rotm[0, 0], rotm[0, 1], rotm[0, 2], q[0]],
                   [rotm[1, 0], rotm[1, 1], rotm[1, 2], q[1]],
                   [rotm[2, 0], rotm[2, 1], rotm[2, 2], q[2]],
                   [0, 0, 0, 1]])

    # Reposition all the points with respect to the inertial frame
    pa = np.matmul(trans, np.transpose(np.append(l1_pos[ar(3)], 1)))
    pb = np.matmul(trans, np.transpose(np.append(l2_pos[ar(3)], 1)))
    pc = np.matmul(trans, np.transpose(np.append(l3_pos[ar(3)], 1)))
    pd = np.matmul(trans, np.transpose(np.append(l4_pos[ar(3)], 1)))
    # Leg 1 (x,y,z)
    p1 = np.matmul(trans, np.transpose(np.append(l1_pos[ar(3) + 3], 1)))
    p2 = np.matmul(trans, np.transpose(np.append(l1_pos[ar(3) + 6], 1)))
    p3 = np.matmul(trans, np.transpose(np.append(l1_pos[ar(3) + 9], 1)))
    # Leg 2 (x,y,z)
    p4 = np.matmul(trans, np.transpose(np.append(l2_pos[ar(3) + 3], 1)))
    p5 = np.matmul(trans, np.transpose(np.append(l2_pos[ar(3) + 6], 1)))
    p6 = np.matmul(trans, np.transpose(np.append(l2_pos[ar(3) + 9], 1)))
    # Leg 3 (x,y,z)
    p7 = np.matmul(trans, np.transpose(np.append(l3_pos[ar(3) + 3], 1)))
    p8 = np.matmul(trans, np.transpose(np.append(l3_pos[ar(3) + 6], 1)))
    p9 = np.matmul(trans, np.transpose(np.append(l3_pos[ar(3) + 9], 1)))
    # Leg 4 (x,y,z)
    p10 = np.matmul(trans, np.transpose(np.append(l4_pos[ar(3) + 3], 1)))
    p11 = np.matmul(trans, np.transpose(np.append(l4_pos[ar(3) + 6], 1)))
    p12 = np.matmul(trans, np.transpose(np.append(l4_pos[ar(3) + 9], 1)))

    plt.plot([pa[0], pb[0], pd[0], pc[0], pa[0]], [pa[1], pb[1], pd[1], pc[1], pa[1]],
             [pa[2], pb[2], pd[2], pc[2], pa[2]], 'yo')
    plt.plot([pa[0], p1[0], p2[0], p3[0]], [pa[1], p1[1], p2[1], p3[1]], [pa[2], p1[2], p2[2], p3[2]], 'k')
    plt.plot([pb[0], p4[0], p5[0], p6[0]], [pb[1], p4[1], p5[1], p6[1]], [pb[2], p4[2], p5[2], p6[2]], 'k')
    plt.plot([pc[0], p7[0], p8[0], p9[0]], [pc[1], p7[1], p8[1], p9[1]], [pc[2], p7[2], p8[2], p9[2]], 'k')
    plt.plot([pd[0], p10[0], p11[0], p12[0]], [pd[1], p10[1], p11[1], p12[1]], [pd[2], p10[2], p11[2], p12[2]], 'k')
    verts = [list(zip([pa[0], pb[0], pd[0], pc[0]], [pa[1], pb[1], pd[1], pc[1]], [pa[2], pb[2], pd[2], pc[2]]))]
    ax.add_collection3d(Poly3DCollection(verts))

    # Plot of COM
    plt.plot([com[0], ], [com[1], ], [com[2], ], '*')  # 3D point
    plt.plot([com[0], ], [com[1], ], [-22.5, ], '*')  # 2D point


if __name__ == '__main__':
    main()
