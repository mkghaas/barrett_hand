#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
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

        self.joint_states = {'bh_j21_joint': 0.0, 'bh_j12_joint': 0.0, 'bh_j22_joint': 0.0, 'bh_j32_joint': 0.0}

        # set up ROS subscriber for real joint angles
        self.sub = rospy.Subscriber('/joint_states', JointState, self.jointStateCallback)

        # set up ROS publisher for desired joint angles
        self.pub = rospy.Publisher("/bhand_node/command", sensor_msgs.msg.JointState, queue_size=10)
        self.msg = sensor_msgs.msg.JointState()
        self.msg.name = ['bh_j11_joint', 'bh_j21_joint', 'bh_j12_joint', 'bh_j22_joint', 'bh_j32_joint']
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

        joint1_label = QLabel("Spread")
        joint2_label = QLabel("Finger 1")
        joint3_label = QLabel("Finger 2")
        joint4_label = QLabel("Thumb")
        
        self.joint1_lineEdit = QLineEdit("0.0")
        self.joint2_lineEdit = QLineEdit("0.0")
        self.joint3_lineEdit = QLineEdit("0.0")
        self.joint4_lineEdit = QLineEdit("0.0")

        self.joint1_lineEdit.setValidator(QDoubleValidator(0.0, 3.2, 2))
        self.joint2_lineEdit.setValidator(QDoubleValidator(0.0, 2.5, 2))
        self.joint3_lineEdit.setValidator(QDoubleValidator(0.0, 2.5, 2))
        self.joint4_lineEdit.setValidator(QDoubleValidator(0.0, 2.5, 2))

        self.joint1_lineEdit.returnPressed.connect(commandButton.click)
        self.joint2_lineEdit.returnPressed.connect(commandButton.click)
        self.joint3_lineEdit.returnPressed.connect(commandButton.click)
        self.joint4_lineEdit.returnPressed.connect(commandButton.click)

        self.joint1_display = QLCDNumber()
        self.joint2_display = QLCDNumber()
        self.joint3_display = QLCDNumber()
        self.joint4_display = QLCDNumber()

        grid.addWidget(joint1_label, 0, 0)
        grid.addWidget(joint2_label, 1, 0)
        grid.addWidget(joint3_label, 2, 0)
        grid.addWidget(joint4_label, 3, 0)

        grid.addWidget(self.joint1_lineEdit, 0, 1)
        grid.addWidget(self.joint2_lineEdit, 1, 1)
        grid.addWidget(self.joint3_lineEdit, 2, 1)
        grid.addWidget(self.joint4_lineEdit, 3, 1)
        grid.addWidget(commandButton, 4, 1)

        grid.addWidget(self.joint1_display, 0, 2)
        grid.addWidget(self.joint2_display, 1, 2)
        grid.addWidget(self.joint3_display, 2, 2)
        grid.addWidget(self.joint4_display, 3, 2)


    def close_fingers_UI(self, grid):

        preloadButton = QPushButton('Close', self)
        preloadButton.clicked.connect(self.sendVelocityCommand)

        self.checkboxes = []
        for i in range(4):
            self.checkboxes.append(QCheckBox())
            grid.addWidget(self.checkboxes[i], i, 4, Qt.AlignCenter)

        grid.addWidget(preloadButton, 4, 4)


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

        self.msg.position[0] = float(self.joint1_lineEdit.text())
        self.msg.position[1] = float(self.joint1_lineEdit.text())
        self.msg.position[2] = float(self.joint2_lineEdit.text())
        self.msg.position[3] = float(self.joint3_lineEdit.text())
        self.msg.position[4] = float(self.joint4_lineEdit.text())
        self.pub.publish(self.msg)
        print "Sent position command:" 
        print self.msg.position[1:5]


    def sendVelocityCommand(self):

        self.callModeServive("VELOCITY")
        if self.checkboxes[0].isChecked:
            self.msg.velocity[0] = 1.0
        else:
            self.msg.velocity[0] = 0.0

        for i in range(4):
            if self.checkboxes[i].isChecked():
                self.msg.velocity[i+1] = 1.0
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

        self.joint1_display.display(self.joint_states['bh_j21_joint'])
        self.joint2_display.display(self.joint_states['bh_j22_joint'])
        self.joint3_display.display(self.joint_states['bh_j12_joint'])
        self.joint4_display.display(self.joint_states['bh_j32_joint'])


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