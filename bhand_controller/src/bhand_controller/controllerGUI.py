#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import time
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import rospy
import sensor_msgs.msg

from bhand_controller.srv import Actions
from bhand_controller.srv import SetControlMode
from sensor_msgs.msg import JointState

class BarrettController(QWidget):
    
    def __init__(self):
        super(BarrettController, self).__init__()

        self.joint_states = {'bh_j21_joint': 0.0, # Spread
                             'bh_j12_joint': 0.0, # Finger 1
                             'bh_j22_joint': 0.0, # Finger 2
                             'bh_j32_joint': 0.0} # Thumb

        # set up ROS subscriber for real joint angles
        self.sub = rospy.Subscriber('/joint_states', JointState, self.jointStateCallback)

        # set up ROS publisher for desired joint angles
        self.pub = rospy.Publisher("/bhand_node/command", sensor_msgs.msg.JointState, queue_size=10)
        self.msg = sensor_msgs.msg.JointState()
        self.msg.name = ['bh_j11_joint',    # Spread 1
                         'bh_j21_joint',    # Spread 2
                         'bh_j12_joint',    # Finger 1
                         'bh_j22_joint',    # Finger 2
                         'bh_j32_joint']    # Thumb
        self.msg.position = [0.0 , 0.0, 0.0, 0.0, 0.0]
        self.msg.velocity = [0.0 , 0.0, 0.0, 0.0, 0.0]
        self.msg.effort = [0.0 , 0.0, 0.0, 0.0, 0.0]

        self.initUI()

        
    def initUI(self):      

        grid = QGridLayout()
        self.set_joint_angles_UI(grid)
        self.close_fingers_UI(grid)
        self.predefined_grasps_UI(grid)
        grid.setColumnMinimumWidth(3, 25)
        grid.setColumnMinimumWidth(5, 25)

        self.setLayout(grid)
        self.setWindowTitle('Barrett control')
        self.show()


    def set_joint_angles_UI(self, grid):

        commandButton = QPushButton('Send', self)
        commandButton.clicked.connect(self.sendPositionCommand)

        labels = [QLabel("Spread"), 
                  QLabel("Finger 1"),
                  QLabel("Finger 2"),
                  QLabel("Thumb")]
        self.jointPosInput = []
        self.jointPosDisplays = []

        for i in range(4):
            self.jointPosInput.append( QLineEdit("0.0") )
            self.jointPosInput[i].setValidator(QDoubleValidator(0.0, 3.14, 2))
            self.jointPosInput[i].returnPressed.connect(commandButton.click)
            self.jointPosDisplays.append( QLCDNumber() )
            grid.addWidget(labels[i], i, 0)
            grid.addWidget(self.jointPosInput[i], i, 1)
            grid.addWidget(self.jointPosDisplays[i], i, 2)
        grid.addWidget(commandButton, 4, 1)


    def close_fingers_UI(self, grid):

        close_finger_button = QPushButton('Close', self)
        close_finger_button.clicked.connect(self.sendVelocityCommand)

        self.checkboxes = []
        for i in range(4):
            self.checkboxes.append(QCheckBox())
            grid.addWidget(self.checkboxes[i], i, 4, Qt.AlignCenter)
        grid.addWidget(close_finger_button, 4, 4)


    def predefined_grasps_UI(self, grid):

        activateButton = QPushButton('Activate', self)
        activateButton.clicked.connect(lambda: self.callActionService(1))

        close_button = QPushButton('Close Grasp', self)
        close_button.clicked.connect(lambda: self.callActionService(2))

        half_close_button = QPushButton('Half-close Grasp', self)
        half_close_button.clicked.connect(lambda: self.callActionService(6))

        open_button = QPushButton('Open Grasp', self)
        open_button.clicked.connect(lambda: self.callActionService(3))

        reset_button = QPushButton('Reset', self)
        reset_button.clicked.connect(lambda: self.callActionService(4))

        grid.addWidget(activateButton, 0, 6)
        grid.addWidget(close_button, 1, 6)
        grid.addWidget(half_close_button, 2, 6)
        grid.addWidget(open_button, 3, 6)
        grid.addWidget(reset_button, 4, 6)


    def sendPositionCommand(self):

        self.callModeServive("POSITION")

        # desired joint position has to be set for both SPREAD1 and SPREAD2
        self.msg.position[0] = float(self.jointPosInput[0].text())
        self.msg.position[1] = float(self.jointPosInput[0].text())
        self.msg.position[2] = float(self.jointPosInput[1].text())
        self.msg.position[3] = float(self.jointPosInput[2].text())
        self.msg.position[4] = float(self.jointPosInput[3].text())
        self.pub.publish(self.msg)
        print "Sent position command:" 
        print self.msg.position[1:5]


    def sendVelocityCommand(self):

        self.callModeServive("VELOCITY")

        # desired velocity has to be set for both SPREAD1 and SPREAD2
        if self.checkboxes[0].isChecked():
            self.msg.velocity[0] = 0.1
        else:
            self.msg.velocity[0] = 0.0

        for i in range(4):
            if self.checkboxes[i].isChecked():
                self.msg.velocity[i+1] = 0.1
            else:
                self.msg.velocity[i+1] = 0.0

        self.pub.publish(self.msg)
        print "Sent velocity command:"
        print self.msg.velocity[1:5]


    def callActionService(self, action):

        try:
            service = rospy.ServiceProxy("/bhand_node/actions", Actions)
            service(action)
        except rospy.ServiceException, e:
            print "Call to activation service failed: %s"%e


    def callModeServive(self, mode):

        try:
            service = rospy.ServiceProxy("/bhand_node/set_control_mode", SetControlMode)
            service(mode)
        except rospy.ServiceException, e:
            print "Call to activation service failed: %s"%e


    def jointStateCallback(self, data):

        for i in range(len(data.name)):
            if self.joint_states.has_key(data.name[i]):
                self.joint_states[data.name[i]] = data.position[i]

        self.jointPosDisplays[0].display(self.joint_states['bh_j21_joint'])
        self.jointPosDisplays[1].display(self.joint_states['bh_j22_joint'])
        self.jointPosDisplays[2].display(self.joint_states['bh_j12_joint'])
        self.jointPosDisplays[3].display(self.joint_states['bh_j32_joint'])


    def keyPressEvent(self, e):

        if e.key() == Qt.Key_Escape:
            self.close()
                
       
def main():
    
    node = rospy.init_node("controllerGUI")

    app = QApplication(sys.argv)
    ex = BarrettController()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()