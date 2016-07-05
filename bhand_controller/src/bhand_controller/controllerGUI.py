#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import rospy
import sensor_msgs.msg

from bhand_controller.srv import Actions
from sensor_msgs.msg import JointState

class BarrettController(QWidget):
    
    def __init__(self):
        super(BarrettController, self).__init__()

        self.joint_states = {'bh_j21_joint': 0.0, 'bh_j32_joint': 0.0, 'bh_j12_joint': 0.0, 'bh_j22_joint': 0.0}

        # set up ROS subscriber for real joint angles
        self.sub = rospy.Subscriber('/joint_states', JointState, self.jointStateCallback)

        # set up ROS publisher for desired joint angles
        self.pub = rospy.Publisher("/bhand_node/command", sensor_msgs.msg.JointState, queue_size=10)
        self.msg = sensor_msgs.msg.JointState()
        self.msg.name = ['bh_j11_joint', 'bh_j21_joint', 'bh_j32_joint', 'bh_j12_joint', 'bh_j22_joint']
        self.msg.position = [0.0 , 0.0, 0.0, 0.0, 0.0]
        self.msg.velocity = [0,0,0,0,0]
        self.msg.effort = [0,0,0,0,0]

        self.initUI()

        
    def initUI(self):      

        activateButton = QPushButton('Activate', self)
        activateButton.clicked.connect(self.activateHand)

        commandButton = QPushButton('Send', self)
        commandButton.clicked.connect(self.sendCommand)
        
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

        grid = QGridLayout()
        grid.addWidget(activateButton, 0, 0)
        grid.addWidget(self.joint1_lineEdit, 1, 0)
        grid.addWidget(self.joint2_lineEdit, 2, 0)
        grid.addWidget(self.joint3_lineEdit, 3, 0)
        grid.addWidget(self.joint4_lineEdit, 4, 0)
        grid.addWidget(commandButton, 5, 0)

        grid.addWidget(self.joint1_display, 1, 1)
        grid.addWidget(self.joint2_display, 2, 1)
        grid.addWidget(self.joint3_display, 3, 1)
        grid.addWidget(self.joint4_display, 4, 1)

        self.setLayout(grid)
        
        self.setWindowTitle('Barrett control')
        self.show()

    def sendCommand(self):

        self.msg.position[0] = float(self.joint1_lineEdit.text())
        self.msg.position[1] = float(self.joint1_lineEdit.text())
        self.msg.position[2] = float(self.joint2_lineEdit.text())
        self.msg.position[3] = float(self.joint3_lineEdit.text())
        self.msg.position[4] = float(self.joint4_lineEdit.text())
        self.pub.publish(self.msg)
        print "Sent Command:" 
        print self.msg.position[1:5]


    def jointStateCallback(self, data):

        for i in range(len(data.name)):
            if self.joint_states.has_key(data.name[i]):
                self.joint_states[data.name[i]] = data.position[i]

        self.joint1_display.display(self.joint_states['bh_j21_joint'])
        self.joint2_display.display(self.joint_states['bh_j32_joint'])
        self.joint3_display.display(self.joint_states['bh_j12_joint'])
        self.joint4_display.display(self.joint_states['bh_j22_joint'])


    def activateHand(self):

        try:
            activateService = rospy.ServiceProxy("/bhand_node/actions", Actions)
            activateService(1)
        except rospy.ServiceException, e:
            print "Call to activation service failed: %s"%e


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