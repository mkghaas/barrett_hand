#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import rospy
import sensor_msgs.msg

from bhand_controller.srv import Actions

class BarrettController(QWidget):
    
    def __init__(self):
        super(BarrettController, self).__init__()

        self.pub = rospy.Publisher("/bhand_node/command", sensor_msgs.msg.JointState, queue_size=10)
        self.msg = sensor_msgs.msg.JointState()
        self.msg.name = ['bh_j11_joint', 'bh_j32_joint', 'bh_j12_joint', 'bh_j22_joint']
        self.msg.position = [0.0 , 0.0, 0.0, 0.0]
        self.msg.velocity = [0,0,0,0]
        self.msg.effort = [0,0,0,0]

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

        self.joint1_lineEdit.setValidator(QDoubleValidator(0.0, 2.0, 2))
        self.joint2_lineEdit.setValidator(QDoubleValidator(0.0, 2.0, 2))
        self.joint3_lineEdit.setValidator(QDoubleValidator(0.0, 2.0, 2))
        self.joint4_lineEdit.setValidator(QDoubleValidator(0.0, 2.0, 2))

        self.joint1_lineEdit.returnPressed.connect(commandButton.click)
        self.joint2_lineEdit.returnPressed.connect(commandButton.click)
        self.joint3_lineEdit.returnPressed.connect(commandButton.click)
        self.joint4_lineEdit.returnPressed.connect(commandButton.click)

        layout = QFormLayout()
        layout.addRow(activateButton)
        layout.addRow("Spread:", self.joint1_lineEdit)
        layout.addRow("Thumb:", self.joint2_lineEdit)
        layout.addRow("Finger 1:", self.joint3_lineEdit)
        layout.addRow("Finger 2:", self.joint4_lineEdit)
        layout.addRow(commandButton)
        self.setLayout(layout)
        
        self.setWindowTitle('Barrett control')
        self.show()

    def sendCommand(self):
        self.msg.position[0] = float(self.joint1_lineEdit.text())
        self.msg.position[1] = float(self.joint2_lineEdit.text())
        self.msg.position[2] = float(self.joint3_lineEdit.text())
        self.msg.position[3] = float(self.joint4_lineEdit.text())
        self.pub.publish(self.msg)
        print "Sent Command:" 
        print self.msg.position

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