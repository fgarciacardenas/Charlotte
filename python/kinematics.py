from __future__ import division
import numpy as np
import time
import sys
# import Adafruit_PCA9685
from cvxopt import matrix, solvers
# Imports for plot
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
# Imports for COM
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
# Import for GUI
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# Commonly used functions
cos = np.cos
sin = np.sin
pi = np.pi
atan2 = np.arctan2
dot = np.dot
array = np.array
ar = np.arange


# GUI
def charly_gui(qf, com, i):
    class App(QMainWindow):
        def __init__(self):
            super().__init__()
            self.left = 10
            self.top = 10
            self.title = 'Charlotte control GUI'
            self.width = 1000
            self.height = 800
            self.initUI()

        def initUI(self):
            self.setWindowTitle(self.title)
            self.setGeometry(self.left, self.top, self.width, self.height)

            m = PlotCanvas(self, width=5, height=4)
            m.move(0, 0)

            self.button = QPushButton('Update position', self)
            self.button.setToolTip('This button updates the graph')
            self.button.move(500, 0)
            self.button.resize(140, 50)

            self.button2 = QPushButton('Update position', self)
            self.button2.setToolTip('This button updates the graph')
            self.button2.move(500, 50)
            self.button2.resize(140, 50)

            self.button3 = QPushButton('Update position', self)
            self.button3.setToolTip('This button updates the graph')
            self.button3.move(500, 100)
            self.button3.resize(140, 50)

            self.lb1 = QLabel('Current configuration', self)
            self.lb1.move(570, 150)
            self.lb1.resize(160, 50)

            self.lb2 = QLabel('Desired configuration', self)
            self.lb2.move(770, 150)
            self.lb2.resize(160, 50)

            self.lbq1 = QLabel(str(qf[i - 1][0]), self)
            self.lbq1.move(570, 200)
            self.lbq1.resize(160, 50)

            self.lbq2 = QLabel(str(qf[i - 1][1]), self)
            self.lbq2.move(570, 220)
            self.lbq2.resize(160, 50)

            self.lbq3 = QLabel(str(qf[i - 1][2]), self)
            self.lbq3.move(570, 240)
            self.lbq3.resize(160, 50)

            self.lbq4 = QLabel(str(qf[i - 1][3]), self)
            self.lbq4.move(570, 260)
            self.lbq4.resize(160, 50)

            self.lbq5 = QLabel(str(qf[i - 1][4]), self)
            self.lbq5.move(570, 280)
            self.lbq5.resize(160, 50)

            self.lbq6 = QLabel(str(qf[i - 1][5]), self)
            self.lbq6.move(570, 300)
            self.lbq6.resize(160, 50)

            self.lbq7 = QLabel(str(qf[i - 1][6]), self)
            self.lbq7.move(570, 320)
            self.lbq7.resize(160, 50)

            self.lbq8 = QLabel(str(qf[i - 1][7]), self)
            self.lbq8.move(570, 340)
            self.lbq8.resize(160, 50)

            self.lbq9 = QLabel(str(qf[i - 1][8]), self)
            self.lbq9.move(570, 360)
            self.lbq9.resize(160, 50)

            self.lbq10 = QLabel(str(qf[i - 1][9]), self)
            self.lbq10.move(570, 380)
            self.lbq10.resize(160, 50)

            self.lbq11 = QLabel(str(qf[i - 1][10]), self)
            self.lbq11.move(570, 400)
            self.lbq11.resize(160, 50)

            self.lbq12 = QLabel(str(qf[i - 1][11]), self)
            self.lbq12.move(570, 420)
            self.lbq12.resize(160, 50)

            self.lbq13 = QLabel(str(qf[i - 1][12]), self)
            self.lbq13.move(570, 440)
            self.lbq13.resize(160, 50)

            self.lbq14 = QLabel(str(qf[i - 1][13]), self)
            self.lbq14.move(570, 460)
            self.lbq14.resize(160, 50)

            self.lbq15 = QLabel(str(qf[i - 1][14]), self)
            self.lbq15.move(570, 480)
            self.lbq15.resize(160, 50)

            self.lbq16 = QLabel(str(qf[i - 1][15]), self)
            self.lbq16.move(570, 500)
            self.lbq16.resize(160, 50)

            self.lbq17 = QLabel(str(qf[i - 1][16]), self)
            self.lbq17.move(570, 520)
            self.lbq17.resize(160, 50)

            self.lbq18 = QLabel(str(qf[i - 1][17]), self)
            self.lbq18.move(570, 540)
            self.lbq18.resize(160, 50)

            # ------------------------------------------------------------------------
            # Position vector
            self.lbq1 = QLabel(str(qf[i - 1][0]), self)
            self.lbq1.move(570, 200)
            self.lbq1.resize(160, 50)

            self.lbq2 = QLabel(str(qf[i - 1][1]), self)
            self.lbq2.move(570, 220)
            self.lbq2.resize(160, 50)

            self.lbq3 = QLabel(str(qf[i - 1][2]), self)
            self.lbq3.move(570, 240)
            self.lbq3.resize(160, 50)

            self.lbq4 = QLabel(str(qf[i - 1][3]), self)
            self.lbq4.move(570, 260)
            self.lbq4.resize(160, 50)

            self.lbq5 = QLabel(str(qf[i - 1][4]), self)
            self.lbq5.move(570, 280)
            self.lbq5.resize(160, 50)

            self.lbq6 = QLabel(str(qf[i - 1][5]), self)
            self.lbq6.move(570, 300)
            self.lbq6.resize(160, 50)

            self.lbq7 = QLabel(str(qf[i - 1][6]), self)
            self.lbq7.move(570, 320)
            self.lbq7.resize(160, 50)

            self.lbq8 = QLabel(str(qf[i - 1][7]), self)
            self.lbq8.move(570, 340)
            self.lbq8.resize(160, 50)

            self.lbq9 = QLabel(str(qf[i - 1][8]), self)
            self.lbq9.move(570, 360)
            self.lbq9.resize(160, 50)

            self.lbq10 = QLabel(str(qf[i - 1][9]), self)
            self.lbq10.move(570, 380)
            self.lbq10.resize(160, 50)

            self.lbq11 = QLabel(str(qf[i - 1][10]), self)
            self.lbq11.move(570, 400)
            self.lbq11.resize(160, 50)

            self.lbq12 = QLabel(str(qf[i - 1][11]), self)
            self.lbq12.move(570, 420)
            self.lbq12.resize(160, 50)

            self.lbq13 = QLabel(str(qf[i - 1][12]), self)
            self.lbq13.move(570, 440)
            self.lbq13.resize(160, 50)

            self.lbq14 = QLabel(str(qf[i - 1][13]), self)
            self.lbq14.move(570, 460)
            self.lbq14.resize(160, 50)

            self.lbq15 = QLabel(str(qf[i - 1][14]), self)
            self.lbq15.move(570, 480)
            self.lbq15.resize(160, 50)

            self.lbq16 = QLabel(str(qf[i - 1][15]), self)
            self.lbq16.move(570, 500)
            self.lbq16.resize(160, 50)

            self.lbq17 = QLabel(str(qf[i - 1][16]), self)
            self.lbq17.move(570, 520)
            self.lbq17.resize(160, 50)

            self.lbq18 = QLabel(str(qf[i - 1][17]), self)
            self.lbq18.move(570, 540)
            self.lbq18.resize(160, 50)

            # ------------------------------------------------------------------------
            # Desired configuration vector
            self.textbox_q1 = QLineEdit(self)
            self.textbox_q1.move(770, 213)
            self.textbox_q1.resize(160, 20)
            # Create a button in the window
            self.button_q1 = QPushButton('Cc', self)
            self.button_q1.move(935, 213)
            self.button_q1.resize(50, 20)
            # connect button to function on_click
            self.button_q1.clicked.connect(self.bt1_click)

            self.textbox_q2 = QLineEdit(self)
            self.textbox_q2.move(770, 233)
            self.textbox_q2.resize(160, 20)
            self.button_q2 = QPushButton('Cc', self)
            self.button_q2.move(935, 233)
            self.button_q2.resize(50, 20)
            self.button_q2.clicked.connect(self.bt2_click)

            self.textbox_q3 = QLineEdit(self)
            self.textbox_q3.move(770, 253)
            self.textbox_q3.resize(160, 20)
            self.button_q3 = QPushButton('Cc', self)
            self.button_q3.move(935, 253)
            self.button_q3.resize(50, 20)
            self.button_q3.clicked.connect(self.bt3_click)

            self.textbox_q4 = QLineEdit(self)
            self.textbox_q4.move(770, 273)
            self.textbox_q4.resize(160, 20)
            self.button_q4 = QPushButton('Cc', self)
            self.button_q4.move(935, 273)
            self.button_q4.resize(50, 20)
            self.button_q4.clicked.connect(self.bt4_click)

            self.textbox_q5 = QLineEdit(self)
            self.textbox_q5.move(770, 293)
            self.textbox_q5.resize(160, 20)
            self.button_q5 = QPushButton('Cc', self)
            self.button_q5.move(935, 293)
            self.button_q5.resize(50, 20)
            self.button_q5.clicked.connect(self.bt5_click)

            self.textbox_q6 = QLineEdit(self)
            self.textbox_q6.move(770, 313)
            self.textbox_q6.resize(160, 20)
            self.button_q6 = QPushButton('Cc', self)
            self.button_q6.move(935, 313)
            self.button_q6.resize(50, 20)
            self.button_q6.clicked.connect(self.bt6_click)

            self.textbox_q7 = QLineEdit(self)
            self.textbox_q7.move(770, 333)
            self.textbox_q7.resize(160, 20)
            self.button_q7 = QPushButton('Cc', self)
            self.button_q7.move(935, 333)
            self.button_q7.resize(50, 20)
            self.button_q7.clicked.connect(self.bt7_click)

            self.textbox_q8 = QLineEdit(self)
            self.textbox_q8.move(770, 353)
            self.textbox_q8.resize(160, 20)
            self.button_q8 = QPushButton('Cc', self)
            self.button_q8.move(935, 353)
            self.button_q8.resize(50, 20)
            self.button_q8.clicked.connect(self.bt8_click)

            self.textbox_q9 = QLineEdit(self)
            self.textbox_q9.move(770, 373)
            self.textbox_q9.resize(160, 20)
            self.button_q9 = QPushButton('Cc', self)
            self.button_q9.move(935, 373)
            self.button_q9.resize(50, 20)
            self.button_q9.clicked.connect(self.bt9_click)

            self.textbox_q10 = QLineEdit(self)
            self.textbox_q10.move(770, 393)
            self.textbox_q10.resize(160, 20)
            self.button_q10 = QPushButton('Cc', self)
            self.button_q10.move(935, 393)
            self.button_q10.resize(50, 20)
            self.button_q10.clicked.connect(self.bt10_click)

            self.textbox_q11 = QLineEdit(self)
            self.textbox_q11.move(770, 413)
            self.textbox_q11.resize(160, 20)
            self.button_q11 = QPushButton('Cc', self)
            self.button_q11.move(935, 413)
            self.button_q11.resize(50, 20)
            self.button_q11.clicked.connect(self.bt11_click)

            self.textbox_q12 = QLineEdit(self)
            self.textbox_q12.move(770, 433)
            self.textbox_q12.resize(160, 20)
            self.button_q12 = QPushButton('Cc', self)
            self.button_q12.move(935, 433)
            self.button_q12.resize(50, 20)
            self.button_q12.clicked.connect(self.bt12_click)

            self.textbox_q13 = QLineEdit(self)
            self.textbox_q13.move(770, 453)
            self.textbox_q13.resize(160, 20)
            self.button_q13 = QPushButton('Cc', self)
            self.button_q13.move(935, 453)
            self.button_q13.resize(50, 20)
            self.button_q13.clicked.connect(self.bt13_click)

            self.textbox_q14 = QLineEdit(self)
            self.textbox_q14.move(770, 473)
            self.textbox_q14.resize(160, 20)
            self.button_q14 = QPushButton('Cc', self)
            self.button_q14.move(935, 473)
            self.button_q14.resize(50, 20)
            self.button_q14.clicked.connect(self.bt14_click)

            self.textbox_q15 = QLineEdit(self)
            self.textbox_q15.move(770, 493)
            self.textbox_q15.resize(160, 20)
            self.button_q15 = QPushButton('Cc', self)
            self.button_q15.move(935, 493)
            self.button_q15.resize(50, 20)
            self.button_q15.clicked.connect(self.bt15_click)

            self.textbox_q16 = QLineEdit(self)
            self.textbox_q16.move(770, 513)
            self.textbox_q16.resize(160, 20)
            self.button_q16 = QPushButton('Cc', self)
            self.button_q16.move(935, 513)
            self.button_q16.resize(50, 20)
            self.button_q16.clicked.connect(self.bt16_click)

            self.textbox_q17 = QLineEdit(self)
            self.textbox_q17.move(770, 533)
            self.textbox_q17.resize(160, 20)
            self.button_q17 = QPushButton('Cc', self)
            self.button_q17.move(935, 533)
            self.button_q17.resize(50, 20)
            self.button_q17.clicked.connect(self.bt17_click)

            self.textbox_q18 = QLineEdit(self)
            self.textbox_q18.move(770, 553)
            self.textbox_q18.resize(160, 20)
            self.button_q18 = QPushButton('Cc', self)
            self.button_q18.move(935, 553)
            self.button_q18.resize(50, 20)
            self.button_q18.clicked.connect(self.bt18_click)

            self.show()

        def bt1_click(self):
             self.textbox_q1.setText(self.lbq1.text())

        def bt2_click(self):
             self.textbox_q2.setText(self.lbq2.text())

        def bt3_click(self):
             self.textbox_q3.setText(self.lbq3.text())

        def bt4_click(self):
             self.textbox_q4.setText(self.lbq4.text())

        def bt5_click(self):
             self.textbox_q5.setText(self.lbq5.text())

        def bt6_click(self):
             self.textbox_q6.setText(self.lbq6.text())

        def bt7_click(self):
             self.textbox_q7.setText(self.lbq7.text())

        def bt8_click(self):
             self.textbox_q8.setText(self.lbq8.text())

        def bt9_click(self):
             self.textbox_q9.setText(self.lbq9.text())

        def bt10_click(self):
             self.textbox_q10.setText(self.lbq10.text())

        def bt11_click(self):
             self.textbox_q11.setText(self.lbq11.text())

        def bt12_click(self):
             self.textbox_q12.setText(self.lbq12.text())

        def bt13_click(self):
             self.textbox_q13.setText(self.lbq13.text())

        def bt14_click(self):
             self.textbox_q14.setText(self.lbq14.text())

        def bt15_click(self):
             self.textbox_q15.setText(self.lbq15.text())

        def bt16_click(self):
             self.textbox_q16.setText(self.lbq16.text())

        def bt17_click(self):
             self.textbox_q17.setText(self.lbq17.text())

        def bt18_click(self):
             self.textbox_q18.setText(self.lbq18.text())


    class PlotCanvas(FigureCanvas):

        def __init__(self, parent=None, width=5, height=4, dpi=100):
            fig = Figure(figsize=(width, height), dpi=dpi)
            self.axes = fig.add_subplot(111, projection='3d')
            FigureCanvas.__init__(self, fig)
            self.setParent(parent)

            FigureCanvas.setSizePolicy(self,
                                       QSizePolicy.Expanding,
                                       QSizePolicy.Expanding)
            FigureCanvas.updateGeometry(self)
            self.plot(qf, com, i)
            self.axes.mouse_init(rotate_btn=1, zoom_btn=3)


        def plot(self, qf, com, i):
            # Simulate the current configuration of the system
            ax = self.figure.add_subplot(111, projection='3d')
            #self.figure.ion()
            for j in range(i):
                #self.axes.cla()
                #self.figure.clf()
                # Manually set the limits
                ax.set_xlim(-40, 40)
                ax.set_ylim(-40, 40)
                ax.set_zlim(-40, 20)

                # Manually label the axes
                ax.set_xlabel('X-axis (cm)')
                ax.set_ylabel('Y-axis (cm)')
                ax.set_zlabel('Z-axis (cm)')

                # Finds and locates the points and vectors for the figure
                #body_simulation(qf[j], ax, com)
                q = qf[j]
                # Find the position of the legs
                l1_pos = fkinematics(q[ar(3) + 3], 1, [q[0], q[1], q[2]], [q[15], q[16], q[17]], 2, 0)
                l2_pos = fkinematics(q[ar(3) + 6], 2, [q[0], q[1], q[2]], [q[15], q[16], q[17]], 2, 0)
                l3_pos = fkinematics(q[ar(3) + 9], 3, [q[0], q[1], q[2]], [q[15], q[16], q[17]], 2, 0)
                l4_pos = fkinematics(q[ar(3) + 12], 4, [q[0], q[1], q[2]], [q[15], q[16], q[17]], 2, 0)

                # Locate the points in the body
                # Leg 1 (x,y,z)
                leg1_base = l1_pos[ar(3)]
                leg1_str = l1_pos[ar(3) + 3]
                leg1_mid = l1_pos[ar(3) + 6]
                leg1_end = l1_pos[ar(3) + 9]
                # Leg 2 (x,y,z)
                leg2_base = l2_pos[ar(3)]
                leg2_str = l2_pos[ar(3) + 3]
                leg2_mid = l2_pos[ar(3) + 6]
                leg2_end = l2_pos[ar(3) + 9]
                # Leg 3 (x,y,z)
                leg3_base = l3_pos[ar(3)]
                leg3_str = l3_pos[ar(3) + 3]
                leg3_mid = l3_pos[ar(3) + 6]
                leg3_end = l3_pos[ar(3) + 9]
                # Leg 4 (x,y,z)
                leg4_base = l4_pos[ar(3)]
                leg4_str = l4_pos[ar(3) + 3]
                leg4_mid = l4_pos[ar(3) + 6]
                leg4_end = l4_pos[ar(3) + 9]

                ax.plot([leg1_base[0], leg2_base[0], leg4_base[0], leg3_base[0], leg1_base[0]],
                         [leg1_base[1], leg2_base[1], leg4_base[1], leg3_base[1], leg1_base[1]],
                         [leg1_base[2], leg2_base[2], leg4_base[2], leg3_base[2], leg1_base[2]], 'yo')
                ax.plot([leg1_base[0], leg1_str[0], leg1_mid[0], leg1_end[0]],
                         [leg1_base[1], leg1_str[1], leg1_mid[1], leg1_end[1]],
                         [leg1_base[2], leg1_str[2], leg1_mid[2], leg1_end[2]], 'k')
                ax.plot([leg2_base[0], leg2_str[0], leg2_mid[0], leg2_end[0]],
                         [leg2_base[1], leg2_str[1], leg2_mid[1], leg2_end[1]],
                         [leg2_base[2], leg2_str[2], leg2_mid[2], leg2_end[2]], 'k')
                ax.plot([leg3_base[0], leg3_str[0], leg3_mid[0], leg3_end[0]],
                         [leg3_base[1], leg3_str[1], leg3_mid[1], leg3_end[1]],
                         [leg3_base[2], leg3_str[2], leg3_mid[2], leg3_end[2]], 'k')
                ax.plot([leg4_base[0], leg4_str[0], leg4_mid[0], leg4_end[0]],
                         [leg4_base[1], leg4_str[1], leg4_mid[1], leg4_end[1]],
                         [leg4_base[2], leg4_str[2], leg4_mid[2], leg4_end[2]], 'k')
                verts = [list(zip([leg1_base[0], leg2_base[0], leg4_base[0], leg3_base[0]],
                                  [leg1_base[1], leg2_base[1], leg4_base[1], leg3_base[1]],
                                  [leg1_base[2], leg2_base[2], leg4_base[2], leg3_base[2]]))]
                ax.add_collection3d(Poly3DCollection(verts))

                # Plot of COM
                ax.plot([com[0], ], [com[1], ], [com[2], ], '*')  # 3D point
                ax.plot([com[0], ], [com[1], ], [-22.5, ], '*')  # 2D point

                ax.figure.canvas.flush_events()
                time.sleep(0.001)
            #self.figure.ioff()
            self.draw()


    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())


def main():
    q = initialize()
    posb = [q[0], q[1], q[2]]
    rotb = [q[15], q[16], q[17]]
    
    # Forward kinematics [leg1,leg2,leg3,leg4]:
    leg1_pos = fkinematics([q[3], q[4], q[5]], 1, posb, rotb,  1, 0)
    leg2_pos = fkinematics([q[6], q[7], q[8]], 2, posb, rotb, 1, 0)
    leg3_pos = fkinematics([q[9], q[10], q[11]], 3, posb, rotb, 1, 0)
    leg4_pos = fkinematics([q[12], q[13], q[14]], 4, posb, rotb, 1, 0)

    # Desired position [posb,leg1,leg2,leg3,leg4,rotb]:
    xd = np.zeros(18)
    xd[0:3] = [2, 1, 0]
    xd[3:6] = array([0, 0, 4]) + leg1_pos
    xd[6:9] = array([3, 0, 0]) + leg2_pos
    xd[9:12] = array([0, 2, 0]) + leg3_pos
    xd[12:15] = array([0, 0, 0]) + leg4_pos
    xd[15:18] = np.radians([0, 15, 0])

    # Desired position of COM
    com_xd = cmass(q)

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
    [qf, i] = quad_prog(q, xd, max_iter, d_t, lamb, w, tol, com_xd, 1)
    # Move servos to desired position
    # move_servo(qf, i, 3)
    # Compare desired and current positions (x,y,z)
    compare(qf, xd, i)
    # Find the center of mass
    [com, __, __] = com_system(q)
    # Simulate the body configuration
    #simulation(qf, com, i)
    charly_gui(qf, com, i)


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


def fkinematics(q, leg, posb, rotb, mode, mode_jacobian):
    # This function finds the forward kinematics of each leg of the robot.
    # Returns the position of the end-effector according to the position and orientation of the base. This is possible
    # by calculating the jacobian of each leg a locating it in a homogeneous transformation matrix.
    global leg_end, leg_base, leg_srt, leg_mid, pos, jacobian
    r1 = 5.5     # Distance from servo 1 to 2
    r2 = 7.5     # Distance from servo 2 to 3
    r3 = 22.5    # Distance from servo 3 to effector
    r4 = 10.253  # Distance from base to servo 1

    # Denavit-Hartenberg matrices
    m_link1 = denavit_hartenberg(q[0], pi/2, r1, 0)
    m_link2 = denavit_hartenberg(q[1] + pi/2, pi, -r2, 0)
    m_link3 = denavit_hartenberg(q[2], pi, -r3, 0)

    # Position of the legs with respect to the base
    repos_leg1 = reposition_leg(-3*pi/4, r4, -r4)
    repos_leg2 = reposition_leg(-pi/4, r4, r4)
    repos_leg3 = reposition_leg(3*pi/4, -r4, -r4)
    repos_leg4 = reposition_leg(pi/4, -r4, r4)

    # Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
    rotm = q2rot(rotb[0], rotb[1], rotb[2])

    trans = array([[rotm[0][0], rotm[0][1], rotm[0][2], posb[0]],
                   [rotm[1][0], rotm[1][1], rotm[1][2], posb[1]],
                   [rotm[2][0], rotm[2][1], rotm[2][2], posb[2]],
                   [0, 0, 0, 1]])

    # Mode 1 returns only the position of the end-effector
    if mode == 1:
        if leg == 1:
            leg_end = trans.dot(repos_leg1).dot(m_link1).dot(m_link2).dot(m_link3)
        elif leg == 2:
            leg_end = trans.dot(repos_leg2).dot(m_link1).dot(m_link2).dot(m_link3)
        elif leg == 3:
            leg_end = trans.dot(repos_leg3).dot(m_link1).dot(m_link2).dot(m_link3)
        elif leg == 4:
            leg_end = trans.dot(repos_leg4).dot(m_link1).dot(m_link2).dot(m_link3)

        # End-effector position (x,y,z)
        pos = array([leg_end[0][3], leg_end[1][3], leg_end[2][3]])

    # Mode 2 returns the position of every link in the system
    elif mode == 2:
        if leg == 1:
            leg_base = trans.dot(repos_leg1)
            leg_srt = trans.dot(repos_leg1).dot(m_link1)
            leg_mid = trans.dot(repos_leg1).dot(m_link1).dot(m_link2)
            leg_end = trans.dot(repos_leg1).dot(m_link1).dot(m_link2).dot(m_link3)
        elif leg == 2:
            leg_base = trans.dot(repos_leg2)
            leg_srt = trans.dot(repos_leg2).dot(m_link1)
            leg_mid = trans.dot(repos_leg2).dot(m_link1).dot(m_link2)
            leg_end = trans.dot(repos_leg2).dot(m_link1).dot(m_link2).dot(m_link3)
        elif leg == 3:
            leg_base = trans.dot(repos_leg3)
            leg_srt = trans.dot(repos_leg3).dot(m_link1)
            leg_mid = trans.dot(repos_leg3).dot(m_link1).dot(m_link2)
            leg_end = trans.dot(repos_leg3).dot(m_link1).dot(m_link2).dot(m_link3)
        elif leg == 4:
            leg_base = trans.dot(repos_leg4)
            leg_srt = trans.dot(repos_leg4).dot(m_link1)
            leg_mid = trans.dot(repos_leg4).dot(m_link1).dot(m_link2)
            leg_end = trans.dot(repos_leg4).dot(m_link1).dot(m_link2).dot(m_link3)

        # Position vector [base, leg_start, leg_middle, leg_end]: (x,y,z)
        p0 = [leg_base[0][3], leg_base[1][3], leg_base[2][3]]
        p1 = [leg_srt[0][3], leg_srt[1][3], leg_srt[2][3]]
        p2 = [leg_mid[0][3], leg_mid[1][3], leg_mid[2][3]]
        p3 = [leg_end[0][3], leg_end[1][3], leg_end[2][3]]
        pos = np.concatenate((p0, p1, p2, p3), axis=0)

    if mode_jacobian == 1:
        if leg == 1:
            jacobian = array([[sin(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3 * sin(q[1] - q[2])),
                               -cos(q[0] + pi / 4) * (r2 * cos(q[1]) + r3 * cos(q[1] - q[2])),
                               r3 * cos(q[1] - q[2]) * cos(q[0] + pi / 4)],
                              [-cos(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3 * sin(q[1] - q[2])),
                               -sin(q[0] + pi / 4) * (r2 * cos(q[1]) + r3 * cos(q[1] - q[2])),
                               r3 * cos(q[1] - q[2]) * sin(q[0] + pi / 4)],
                              [0, r2 * sin(q[1]) + r3 * sin(q[1] - q[2]), -r3 * sin(q[1] - q[2])]])

        elif leg == 2:
            jacobian = array([[cos(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3 * sin(q[1] - q[2])),
                               sin(q[0] + pi / 4) * (r2 * cos(q[1]) + r3 * cos(q[1] - q[2])),
                               -r3 * cos(q[1] - q[2]) * sin(q[0] + pi / 4)],
                              [sin(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3 * sin(q[1] - q[2])),
                               -cos(q[0] + pi / 4) * (r2 * cos(q[1]) + r3 * cos(q[1] - q[2])),
                               r3 * cos(q[1] - q[2]) * cos(q[0] + pi / 4)],
                              [0, r2 * sin(q[1]) + r3 * sin(q[1] - q[2]), -r3 * sin(q[1] - q[2])]])

        elif leg == 3:
            jacobian = array([[-cos(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3 * sin(q[1] - q[2])),
                               -sin(q[0] + pi / 4) * (r2 * cos(q[1]) + r3 * cos(q[1] - q[2])),
                               r3 * cos(q[1] - q[2]) * sin(q[0] + pi / 4)],
                              [-sin(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3 * sin(q[1] - q[2])),
                               cos(q[0] + pi / 4) * (r2 * cos(q[1]) + r3 * cos(q[1] - q[2])),
                               -r3 * cos(q[1] - q[2]) * cos(q[0] + pi / 4)],
                              [0, r2 * sin(q[1]) + r3 * sin(q[1] - q[2]), -r3 * sin(q[1] - q[2])]])

        elif leg == 4:
            jacobian = array([[-sin(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3 * sin(q[1] - q[2])),
                               cos(q[0] + pi / 4) * (r2 * cos(q[1]) + r3 * cos(q[1] - q[2])),
                               -r3 * cos(q[1] - q[2]) * cos(q[0] + pi / 4)],
                              [cos(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3 * sin(q[1] - q[2])),
                               sin(q[0] + pi / 4) * (r2 * cos(q[1]) + r3 * cos(q[1] - q[2])),
                               -r3 * cos(q[1] - q[2]) * sin(q[0] + pi / 4)],
                              [0, r2 * sin(q[1]) + r3 * sin(q[1] - q[2]), -r3 * sin(q[1] - q[2])]])

        jacobian = rotm.dot(jacobian)
        return [pos, jacobian]

    return pos


def denavit_hartenberg(theta, alpha, r, d):
    mat = array([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), r * cos(theta)],
                 [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta)],
                 [0, sin(alpha), cos(alpha), d],
                 [0, 0, 0, 1]])
    return mat


def reposition_leg(ang, dx, dy):
    leg_pos = array([[cos(ang), -sin(ang), 0, dx],
                     [sin(ang), cos(ang), 0, dy],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
    return leg_pos


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
    a = atan2(rotm[2][1], rotm[2][2])
    b = atan2(-rotm[2][0], np.sqrt((rotm[2][1] ** 2) + (rotm[2][2] ** 2)))
    c = atan2(rotm[1][0], rotm[0][0])

    return [a, b, c]


def quad_prog(q, xd, max_iter, d_t, lamb, w, tol, com_xd, mode):
    # This function manages the minimization program and find the error of the desired function
    qf = np.zeros((max_iter, 18))
    i = 0

    while i < max_iter:
        # Quadratic programming
        [h, f] = costfunc(q, xd, lamb, w, com_xd, mode)
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
        dq = array(sol['x'])

        # Update the position vector
        q[ar(15)] = q[ar(15)] + d_t * dq[ar(15)].T

        # Update the orientation vector
        rot_axis = dot(array([[cos(q[16]) * cos(q[17]), -sin(q[17]), 0],
                              [cos(q[16]) * sin(q[17]), cos(q[17]), 0],
                              [-sin(q[16]), 0, 1]]), [dq[15], dq[16], dq[17]])

        skw = array([[0, -rot_axis[2], rot_axis[1]],
                     [rot_axis[2], 0, -rot_axis[0]],
                     [-rot_axis[1], rot_axis[0], 0]])

        # Rodrigues rotation formula
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


def costfunc(q, xd, lamb, w, com_xd, mode):
    # This function finds the values of h and f in order to initialize the quadratic program.
    # The inputs are q: actuated and sub-actuated angles, xd: desired position vector, p: weights and lamb: gain.

    # Position and orientation of the base
    posb = array([q[0], q[1], q[2]])
    rotb = array([q[15], q[16], q[17]])

    # Position and jacobian of each leg
    [pos1, j1] = leg_jacobian(q[3:6], 1, posb, rotb)
    [pos2, j2] = leg_jacobian(q[6:9], 2, posb, rotb)
    [pos3, j3] = leg_jacobian(q[9:12], 3, posb, rotb)
    [pos4, j4] = leg_jacobian(q[12:15], 4, posb, rotb)

    # Mode 1 = Free-floating base control
    if mode == 1:
        # Jacobians of the base position and orientation
        j5 = np.concatenate((np.eye(3), np.zeros((3, 15))), axis=1)
        j6 = np.concatenate((np.zeros((3, 15)), np.eye(3)), axis=1)

        # Values of h and f (Hessian and vector of linear elements):
        h = w[0] * j1.T.dot(j1) + w[1] * j2.T.dot(j2) + w[2] * j3.T.dot(j3) + w[3] * j4.T.dot(j4) + w[4] * j5.T.dot(
            j5) + w[5] * j6.T.dot(j6)

        f = -2 * (w[1] * lamb * (pos1 - xd[3:6]).T.dot(j1) + w[2] * lamb * (pos2 - xd[6:9]).T.dot(j2) + w[3] * lamb * (
                    pos3 - xd[9:12]).T.dot(j3) + w[4] * lamb * (pos4 - xd[12:15]).T.dot(j4) + w[0] * lamb * (
                              posb - xd[0:3]).T.dot(j5) + w[5] * lamb * (rotb - xd[15:18]).T.dot(j6))

        return [h, f]

    # Mode 2 = COM base control
    elif mode == 2:
        # Jacobians of center of mass
        [com, j_com, __] = com_system(q)

        # Values of h and f (Hessian and vector of linear elements):
        h = w[0] * j1.T.dot(j1) + w[1] * j2.T.dot(j2) + w[2] * j3.T.dot(j3) + w[3] * j4.T.dot(j4) + w[4] * j_com.T.dot(
            j_com)

        f = -2 * (w[1] * lamb * (pos1 - xd[3:6]).T.dot(j1) + w[2] * lamb * (pos2 - xd[6:9]).T.dot(j2) + w[3] * lamb * (
                pos3 - xd[9:12]).T.dot(j3) + w[4] * lamb * (pos4 - xd[12:15]).T.dot(j4) + w[0] * lamb * (
                          com - com_xd).T.dot(j_com))

        return [h, f]


def leg_jacobian(q, leg, posb, rotb):
    global leg_ja
    [leg_pos, end_jacobian] = fkinematics(q, leg, posb, rotb, 1, 1)
    # Distance vector of the end-effector with respect to base
    d = posb - pos
    # Skew-Symmetric matrix
    m_skew = array([[0, -d[2], d[1]],
                    [d[2], 0, -d[0]],
                    [-d[1], d[0], 0]])
    if leg == 1:
        leg_ja = np.concatenate((np.eye(3), end_jacobian, np.zeros((3, 9)), m_skew), axis=1)
    elif leg == 2:
        leg_ja = np.concatenate((np.eye(3), np.zeros((3, 3)), end_jacobian, np.zeros((3, 6)), m_skew), axis=1)
    elif leg == 3:
        leg_ja = np.concatenate((np.eye(3), np.zeros((3, 6)), end_jacobian, np.zeros((3, 3)), m_skew), axis=1)
    elif leg == 4:
        leg_ja = np.concatenate((np.eye(3), np.zeros((3, 9)), end_jacobian, m_skew), axis=1)

    return [leg_pos, leg_ja]


def calc_err(q, xd):
    # Find the current position
    posb = [q[0], q[1], q[2]]
    rotb = [q[15], q[16], q[17]]
    leg1f = fkinematics([q[3], q[4], q[5]], 1, posb, rotb, 1, 0)
    leg2f = fkinematics([q[6], q[7], q[8]], 2, posb, rotb, 1, 0)
    leg3f = fkinematics([q[9], q[10], q[11]], 3, posb, rotb, 1, 0)
    leg4f = fkinematics([q[12], q[13], q[14]], 4, posb, rotb, 1, 0)

    # Find the error of each leg and the base
    err_leg1 = xd[ar(3) + 3] - leg1f
    err_leg2 = xd[ar(3) + 6] - leg2f
    err_leg3 = xd[ar(3) + 9] - leg3f
    err_leg4 = xd[ar(3) + 12] - leg4f
    err_posb = xd[ar(3)] - q[ar(3)]
    err_rotb = xd[ar(3) + 15] - q[ar(3) + 15]

    # Sum of the squared errors
    err = np.sqrt(
        err_leg1[0] ** 2 + err_leg1[1] ** 2 + err_leg1[2] ** 2 + err_leg2[0] ** 2 + err_leg2[1] ** 2 + err_leg2[
            2] ** 2 + err_leg3[0] ** 2 +
        err_leg3[1] ** 2 + err_leg3[2] ** 2 + err_leg4[0] ** 2 + err_leg4[1] ** 2 + err_leg4[2] ** 2 + err_posb[
            0] ** 2 + err_posb[1] ** 2 +
        err_posb[2] ** 2 + err_rotb[0] ** 2 + err_rotb[1] ** 2 + err_rotb[2] ** 2)

    return err


def cmass(q):
    # Determines if the center of mass (COM) is in a stable configuration
    [com, __, feet] = com_system(q)
    # Projects the COM in the ground
    point = Point(com[0], com[1])
    # Generate point
    p1 = array(feet[ar(2)])
    p2 = array(feet[ar(2)+3])
    p3 = array(feet[ar(2)+6])
    p4 = array(feet[ar(2)+9])
    p5 = array([com[0], com[1]])

    # Finds if the point is inside the polygon
    if feet[2] < -22.6 or feet[2] > -22.4:
        polygon = Polygon([p2, p3, p4])
        grad1 = point_to_line_dist(p5, [p2, p3], 0)
        grad2 = point_to_line_dist(p5, [p2, p4], 0)
        grad3 = point_to_line_dist(p5, [p3, p4], 0)
        ind = np.argmin([grad1, grad2, grad3])
        if ind == 0:
            [x_com, y_com] = point_to_line_dist(p5, [p2, p3], 1)
        elif ind == 1:
            [x_com, y_com] = point_to_line_dist(p5, [p2, p4], 1)
        elif ind == 2:
            [x_com, y_com] = point_to_line_dist(p5, [p3, p4], 1)
    elif feet[5] < -22.6 or feet[5] > -22.4:
        polygon = Polygon([p1, p3, p4])
        grad1 = point_to_line_dist(p5, [p1, p3], 0)
        grad2 = point_to_line_dist(p5, [p1, p4], 0)
        grad3 = point_to_line_dist(p5, [p3, p4], 0)
        ind = np.argmin([grad1, grad2, grad3])
        if ind == 0:
            [x_com, y_com] = point_to_line_dist(p5, [p1, p3], 1)
        elif ind == 1:
            [x_com, y_com] = point_to_line_dist(p5, [p1, p4], 1)
        elif ind == 2:
            [x_com, y_com] = point_to_line_dist(p5, [p3, p4], 1)
    elif feet[8] < -22.6 or feet[8] > -22.4:
        polygon = Polygon([p1, p2, p4])
        grad1 = point_to_line_dist(p5, [p1, p2], 0)
        grad2 = point_to_line_dist(p5, [p1, p4], 0)
        grad3 = point_to_line_dist(p5, [p2, p4], 0)
        ind = np.argmin([grad1, grad2, grad3])
        if ind == 0:
            [x_com, y_com] = point_to_line_dist(p5, [p1, p2], 1)
        elif ind == 1:
            [x_com, y_com] = point_to_line_dist(p5, [p1, p4], 1)
        elif ind == 2:
            [x_com, y_com] = point_to_line_dist(p5, [p2, p4], 1)
    elif feet[11] < -22.6 or feet[11] > -22.4:
        polygon = Polygon([p1, p2, p3])
        grad1 = point_to_line_dist(p5, [p1, p2], 0)
        grad2 = point_to_line_dist(p5, [p1, p3], 0)
        grad3 = point_to_line_dist(p5, [p2, p3], 0)
        ind = np.argmin([grad1, grad2, grad3])
        if ind == 0:
            [x_com, y_com] = point_to_line_dist(p5, [p1, p2], 1)
        elif ind == 1:
            [x_com, y_com] = point_to_line_dist(p5, [p1, p3], 1)
        elif ind == 2:
            [x_com, y_com] = point_to_line_dist(p5, [p2, p3], 1)
    else:
        polygon = Polygon([feet[ar(2)], feet[ar(2) + 3], feet[ar(2) + 9], feet[ar(2) + 6]])
        x_com = 0
        y_com = 0
    print('Is the point inside the polygon?: ', polygon.contains(point))

    com_xd = array([x_com, y_com, com[2]])

    return com_xd


def com_system(q):
    # Weights
    w_com1 = 0.075
    w_com2 = 0.15
    w_com3 = 0.2
    w_base = 0.7
    w_total = 4*w_com1 + 4*w_com2 + 4*w_com3 + w_base
    w = [w_com1, w_com2, w_com3, w_total]

    # Find the center of mass
    [leg1_com, j1_com2, j1_com3] = com_kinematics(q[ar(3) + 3], 1, q[ar(3)], q[ar(3) + 15], w)
    [leg2_com, j2_com2, j2_com3] = com_kinematics(q[ar(3) + 6], 2, q[ar(3)], q[ar(3) + 15], w)
    [leg3_com, j3_com2, j3_com3] = com_kinematics(q[ar(3) + 9], 3, q[ar(3)], q[ar(3) + 15], w)
    [leg4_com, j4_com2, j4_com3] = com_kinematics(q[ar(3) + 12], 4, q[ar(3)], q[ar(3) + 15], w)

    # COM of the base
    base = w_base * array([0, 0, 0]) / w_total

    # COM position
    com = leg1_com[ar(3)] + leg1_com[ar(3) + 3] + leg1_com[ar(3) + 6] + leg2_com[ar(3)] + \
          leg2_com[ar(3) + 3] + leg2_com[ar(3) + 6] + leg3_com[ar(3)] + leg3_com[ar(3) + 3] + \
          leg3_com[ar(3) + 6] + leg4_com[ar(3)] + leg4_com[ar(3) + 3] + leg4_com[ar(3) + 6] + base

    # COM jacobian
    j_com = j1_com2 + j1_com3 + j2_com2 + j2_com3 + j3_com2 + j3_com3 + j4_com2 + j4_com3

    # Location of the legs
    f1 = [leg1_com[9], leg1_com[10], leg1_com[11]]
    f2 = [leg2_com[9], leg2_com[10], leg2_com[11]]
    f3 = [leg3_com[9], leg3_com[10], leg3_com[11]]
    f4 = [leg4_com[9], leg4_com[10], leg4_com[11]]
    feet = np.concatenate((f1, f2, f3, f4), axis=0)

    return [com, j_com, feet]


def com_kinematics(q, leg, posb, rotb, w):
    # Variables
    global mf, m_com3, m_com2, m_com1, jacobian_com2, jacobian_com3
    r1 = 5.5     # Distance from servo 1 to 2
    r2t = 3.75   # Distance from servo 2 to com2
    r2 = 7.5     # Distance from servo 2 to 3
    r3t = 11     # Distance from servo 3 to com3
    r3 = 22.5    # Distance from servo 3 to end-effector
    r4 = 10.253  # Distance from base to servo 1

    # Weights
    w_com1 = w[0]
    w_com2 = w[1]
    w_com3 = w[2]
    w_total = w[3]

    # Denavit-hartenberg matrices
    m_1 = denavit_hartenberg(q[0], pi/2, r1, 0)
    m_2t = denavit_hartenberg(q[1] + pi / 2, pi, -r2t, 0)
    m_2 = denavit_hartenberg(q[1] + pi/2, pi, -r2, 0)
    m_3t = denavit_hartenberg(q[2], pi, -r3t, 0)
    m_3 = denavit_hartenberg(q[2], pi, -r3, 0)

    # Position of the legs with respect to the base
    repos_leg1 = reposition_leg(-3 * pi / 4, r4, -r4)
    repos_leg2 = reposition_leg(-pi / 4, r4, r4)
    repos_leg3 = reposition_leg(3 * pi / 4, -r4, -r4)
    repos_leg4 = reposition_leg(pi / 4, -r4, r4)

    # Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
    rotm = q2rot(rotb[0], rotb[1], rotb[2])

    trans = array([[rotm[0][0], rotm[0][1], rotm[0][2], posb[0]],
                   [rotm[1][0], rotm[1][1], rotm[1][2], posb[1]],
                   [rotm[2][0], rotm[2][1], rotm[2][2], posb[2]],
                   [0, 0, 0, 1]])

    # Location of center of mass
    if leg == 1:
        # Position and weight of the center of mass
        m_com1 = (w_com1 / w_total) * trans.dot(repos_leg1)
        m_com2 = (w_com2 / w_total) * trans.dot(repos_leg1).dot(m_1).dot(m_2t)
        m_com3 = (w_com3 / w_total) * trans.dot(repos_leg1).dot(m_1).dot(m_2).dot(m_3t)
        mf = trans.dot(repos_leg1).dot(m_1).dot(m_2).dot(m_3)

        # Jacobian of center of mass
        jacobian_com2 = array([[sin(q[0] + pi / 4) * (r1 + r2t * sin(q[1])), -r2t * cos(q[1]) * cos(q[0] + pi / 4), 0],
                               [-cos(q[0] + pi / 4) * (r1 + r2t * sin(q[1])), -r2t * cos(q[1]) * sin(q[0] + pi / 4), 0],
                               [0, r2t * sin(q[1]), 0]])

        jacobian_com3 = array([[sin(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3t * sin(q[1] - q[2])),
                                -cos(q[0] + pi / 4) * (r2 * cos(q[1]) + r3t * cos(q[1] - q[2])),
                                r3t * cos(q[1] - q[2]) * cos(q[0] + pi / 4)],
                               [-cos(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3t * sin(q[1] - q[2])),
                                -sin(q[0] + pi / 4) * (
                                        r2 * cos(q[1]) + r3t * cos(q[1] - q[2])),
                                r3t * cos(q[1] - q[2]) * sin(q[0] + pi / 4)],
                               [0, r2 * sin(q[1]) + r3t * sin(q[1] - q[2]), -r3t * sin(q[1] - q[2])]])

    elif leg == 2:
        # Position and weight of the center of mass
        m_com1 = (w_com1 / w_total) * trans.dot(repos_leg2)
        m_com2 = (w_com2 / w_total) * trans.dot(repos_leg2).dot(m_1).dot(m_2t)
        m_com3 = (w_com3 / w_total) * trans.dot(repos_leg2).dot(m_1).dot(m_2).dot(m_3t)
        mf = trans.dot(repos_leg2).dot(m_1).dot(m_2).dot(m_3)

        # Jacobian of center of mass
        jacobian_com2 = array([[cos(q[0] + pi / 4) * (r1 + r2t * sin(q[1])), r2t * cos(q[1]) * sin(q[0] + pi / 4), 0],
                               [sin(q[0] + pi / 4) * (r1 + r2t * sin(q[1])), -r2t * cos(q[1]) * cos(q[0] + pi / 4), 0],
                               [0, r2t * sin(q[1]), 0]])

        jacobian_com3 = array([[cos(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3t * sin(q[1] - q[2])),
                                sin(q[0] + pi / 4) * (r2 * cos(q[1]) + r3t * cos(q[1] - q[2])),
                                -r3t * cos(q[1] - q[2]) * sin(q[0] + pi / 4)],
                               [sin(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3t * sin(q[1] - q[2])),
                                -cos(q[0] + pi / 4) * (
                                        r2 * cos(q[1]) + r3t * cos(q[1] - q[2])),
                                r3t * cos(q[1] - q[2]) * cos(q[0] + pi / 4)],
                               [0, r2 * sin(q[1]) + r3t * sin(q[1] - q[2]), -r3t * sin(q[1] - q[2])]])

    elif leg == 3:
        # Position and weight of the center of mass
        m_com1 = (w_com1 / w_total) * trans.dot(repos_leg3)
        m_com2 = (w_com2 / w_total) * trans.dot(repos_leg3).dot(m_1).dot(m_2t)
        m_com3 = (w_com3 / w_total) * trans.dot(repos_leg3).dot(m_1).dot(m_2).dot(m_3t)
        mf = trans.dot(repos_leg3).dot(m_1).dot(m_2).dot(m_3)

        # Jacobian of center of mass
        jacobian_com2 = array([[-cos(q[0] + pi / 4) * (r1 + r2t * sin(q[1])), -r2t * cos(q[1]) * sin(q[0] + pi / 4), 0],
                               [-sin(q[0] + pi / 4) * (r1 + r2t * sin(q[1])), r2t * cos(q[1]) * cos(q[0] + pi / 4), 0],
                               [0, r2t * sin(q[1]), 0]])

        jacobian_com3 = array([[-cos(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3t * sin(q[1] - q[2])),
                                -sin(q[0] + pi / 4) * (r2 * cos(q[1]) + r3t * cos(q[1] - q[2])),
                                r3t * cos(q[1] - q[2]) * sin(q[0] + pi / 4)],
                               [-sin(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3t * sin(q[1] - q[2])),
                                cos(q[0] + pi / 4) * (
                                        r2 * cos(q[1]) + r3t * cos(q[1] - q[2])),
                                -r3t * cos(q[1] - q[2]) * cos(q[0] + pi / 4)],
                               [0, r2 * sin(q[1]) + r3t * sin(q[1] - q[2]), -r3t * sin(q[1] - q[2])]])

    elif leg == 4:
        # Position and weight of the center of mass
        m_com1 = (w_com1 / w_total) * trans.dot(repos_leg4)
        m_com2 = (w_com2 / w_total) * trans.dot(repos_leg4).dot(m_1).dot(m_2t)
        m_com3 = (w_com3 / w_total) * trans.dot(repos_leg4).dot(m_1).dot(m_2).dot(m_3t)
        mf = trans.dot(repos_leg4).dot(m_1).dot(m_2).dot(m_3)

        # Jacobian of center of mass
        jacobian_com2 = array([[-sin(q[0] + pi / 4) * (r1 + r2t * sin(q[1])), r2t * cos(q[1]) * cos(q[0] + pi / 4), 0],
                               [cos(q[0] + pi / 4) * (r1 + r2t * sin(q[1])), r2t * cos(q[1]) * sin(q[0] + pi / 4), 0],
                               [0, r2t * sin(q[1]), 0]])

        jacobian_com3 = array([[-sin(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3t * sin(q[1] - q[2])),
                                cos(q[0] + pi / 4) * (r2 * cos(q[1]) + r3t * cos(q[1] - q[2])),
                                -r3t * cos(q[1] - q[2]) * cos(q[0] + pi / 4)],
                               [cos(q[0] + pi / 4) * (r1 + r2 * sin(q[1]) + r3t * sin(q[1] - q[2])),
                                sin(q[0] + pi / 4) * (r2 * cos(q[1]) + r3t * cos(q[1] - q[2])),
                                -r3t * cos(q[1] - q[2]) * sin(q[0] + pi / 4)],
                               [0, r2 * sin(q[1]) + r3t * sin(q[1] - q[2]), -r3t * sin(q[1] - q[2])]])

    # Reposition jacobian
    jacobian_com2 = rotm.dot(jacobian_com2) * (w_com2 / w_total)
    jacobian_com3 = rotm.dot(jacobian_com3) * (w_com3 / w_total)

    # Position Vector
    p1 = [m_com1[0][3], m_com1[1][3], m_com1[2][3]]
    p2 = [m_com2[0][3], m_com2[1][3], m_com2[2][3]]
    p3 = [m_com3[0][3], m_com3[1][3], m_com3[2][3]]
    p4 = [mf[0][3], mf[1][3], mf[2][3]]
    position = np.concatenate((p1, p2, p3, p4), axis=0)

    return [position, jacobian_com2, jacobian_com3]


def point_to_line_dist(point, line, mode):
    # Unit vector
    unit_line = line[1] - line[0]
    norm_unit_line = unit_line / np.linalg.norm(unit_line)

    # Compute the perpendicular distance to the theoretical infinite line
    segment_dist = (
        np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) /
        np.linalg.norm(unit_line)
    )

    diff = (
        (norm_unit_line[0] * (point[0] - line[0][0])) +
        (norm_unit_line[1] * (point[1] - line[0][1]))
    )

    x_seg = (norm_unit_line[0] * diff) + line[0][0]
    y_seg = (norm_unit_line[1] * diff) + line[0][1]

    endpoint_dist = min(
        np.linalg.norm(line[0] - point),
        np.linalg.norm(line[1] - point)
    )

    # Decide if the intersection point falls on the line segment
    lp1_x = line[0][0]  # Line point 1 x
    lp1_y = line[0][1]  # Line point 1 y
    lp2_x = line[1][0]  # Line point 2 x
    lp2_y = line[1][1]  # Line point 2 y
    is_betw_x = lp1_x <= x_seg <= lp2_x or lp2_x <= x_seg <= lp1_x
    is_betw_y = lp1_y <= y_seg <= lp2_y or lp2_y <= y_seg <= lp1_y

    if mode == 1:
        if is_betw_x and is_betw_y:
            return [x_seg, y_seg]
        else:
            # If not, then return the minimum distance to the segment endpoints
            ind = np.argmin([np.linalg.norm(line[0] - point), np.linalg.norm(line[1] - point)])
            if ind == 0:
                return [line[0][0], line[0][1]]
            elif ind == 1:
                return [line[1][0], line[1][1]]
    else:
        if is_betw_x and is_betw_y:
            return segment_dist
        else:
            # If not, then return the minimum distance to the segment endpoints
            return endpoint_dist


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


def body_simulation(q, ax, com):
    # Find the position of the legs
    l1_pos = fkinematics(q[ar(3) + 3], 1, [q[0], q[1], q[2]], [q[15], q[16], q[17]], 2, 0)
    l2_pos = fkinematics(q[ar(3) + 6], 2, [q[0], q[1], q[2]], [q[15], q[16], q[17]], 2, 0)
    l3_pos = fkinematics(q[ar(3) + 9], 3, [q[0], q[1], q[2]], [q[15], q[16], q[17]], 2, 0)
    l4_pos = fkinematics(q[ar(3) + 12], 4, [q[0], q[1], q[2]], [q[15], q[16], q[17]], 2, 0)

    # Locate the points in the body
    # Leg 1 (x,y,z)
    leg1_base = l1_pos[ar(3)]
    leg1_str = l1_pos[ar(3) + 3]
    leg1_mid = l1_pos[ar(3) + 6]
    leg1_end = l1_pos[ar(3) + 9]
    # Leg 2 (x,y,z)
    leg2_base = l2_pos[ar(3)]
    leg2_str = l2_pos[ar(3) + 3]
    leg2_mid = l2_pos[ar(3) + 6]
    leg2_end = l2_pos[ar(3) + 9]
    # Leg 3 (x,y,z)
    leg3_base = l3_pos[ar(3)]
    leg3_str = l3_pos[ar(3) + 3]
    leg3_mid = l3_pos[ar(3) + 6]
    leg3_end = l3_pos[ar(3) + 9]
    # Leg 4 (x,y,z)
    leg4_base = l4_pos[ar(3)]
    leg4_str = l4_pos[ar(3) + 3]
    leg4_mid = l4_pos[ar(3) + 6]
    leg4_end = l4_pos[ar(3) + 9]

    plt.plot([leg1_base[0], leg2_base[0], leg4_base[0], leg3_base[0], leg1_base[0]],
             [leg1_base[1], leg2_base[1], leg4_base[1], leg3_base[1], leg1_base[1]],
             [leg1_base[2], leg2_base[2], leg4_base[2], leg3_base[2], leg1_base[2]], 'yo')
    plt.plot([leg1_base[0], leg1_str[0], leg1_mid[0], leg1_end[0]],
             [leg1_base[1], leg1_str[1], leg1_mid[1], leg1_end[1]],
             [leg1_base[2], leg1_str[2], leg1_mid[2], leg1_end[2]], 'k')
    plt.plot([leg2_base[0], leg2_str[0], leg2_mid[0], leg2_end[0]],
             [leg2_base[1], leg2_str[1], leg2_mid[1], leg2_end[1]],
             [leg2_base[2], leg2_str[2], leg2_mid[2], leg2_end[2]], 'k')
    plt.plot([leg3_base[0], leg3_str[0], leg3_mid[0], leg3_end[0]],
             [leg3_base[1], leg3_str[1], leg3_mid[1], leg3_end[1]],
             [leg3_base[2], leg3_str[2], leg3_mid[2], leg3_end[2]], 'k')
    plt.plot([leg4_base[0], leg4_str[0], leg4_mid[0], leg4_end[0]],
             [leg4_base[1], leg4_str[1], leg4_mid[1], leg4_end[1]],
             [leg4_base[2], leg4_str[2], leg4_mid[2], leg4_end[2]], 'k')
    verts = [list(zip([leg1_base[0], leg2_base[0], leg4_base[0], leg3_base[0]],
                      [leg1_base[1], leg2_base[1], leg4_base[1], leg3_base[1]],
                      [leg1_base[2], leg2_base[2], leg4_base[2], leg3_base[2]]))]
    ax.add_collection3d(Poly3DCollection(verts))

    # Plot of COM
    plt.plot([com[0], ], [com[1], ], [com[2], ], '*')  # 3D point
    plt.plot([com[0], ], [com[1], ], [-22.5, ], '*')  # 2D point


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
    rotb = [qf[i - 1][15], qf[i - 1][16], qf[i - 1][17]]
    leg1f = fkinematics([qf[i - 1][3], qf[i - 1][4], qf[i - 1][5]], 1, posb, rotb, 1, 0)
    leg2f = fkinematics([qf[i - 1][6], qf[i - 1][7], qf[i - 1][8]], 2, posb, rotb, 1, 0)
    leg3f = fkinematics([qf[i - 1][9], qf[i - 1][10], qf[i - 1][11]], 3, posb, rotb, 1, 0)
    leg4f = fkinematics([qf[i - 1][12], qf[i - 1][13], qf[i - 1][14]], 4, posb, rotb, 1, 0)

    print("Desired pos:", xd[ar(3) + 3], "\n", "           ", xd[ar(3) + 6])
    print("            ", xd[ar(3) + 9], "\n", "           ", xd[ar(3) + 12])
    print("Final pos  :", np.around(leg1f, 8), "\n", "           ", np.around(leg2f, 8))
    print("            ", np.around(leg3f, 8), "\n", "           ", np.around(leg4f, 8))
    print("Body pos   :", xd[ar(3)], qf[i - 1][ar(3)])
    print("Body rot   :", xd[ar(3) + 15], qf[i - 1][ar(3) + 15])


if __name__ == '__main__':
    main()
