#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GUI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import threading
from both_arm_head.msg import Num


rospy.init_node('GUI_TOMO')
pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=5)


try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):

    def __init__(self,pub):

        self.pub = pub
        
    def Get_angle(self):
        print("Choose button is already clicked")
        
        joint_0 = float(self.textEdit.toPlainText())
        joint_1 = float(self.textEdit_2.toPlainText())
        joint_2 = float(self.textEdit_3.toPlainText())
        joint_3 = float(self.textEdit_4.toPlainText())
        joint_4 = float(self.textEdit_5.toPlainText())
        joint_5 = float(self.textEdit_6.toPlainText())

        print(joint_1 * joint_2)
        print(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        # self.textEdit_2.setText(a)
        ########################################3
        hello_str = JointState()
        hello_str.header = Header()
        hello_str.name = ['head_1','head_2','joint_1_right', 'joint_2_right', 'joint_3_right', 'joint_4_right','joint_5_right','hand_joint','joint_1_left', 'joint_2_left', 'joint_3_left', 'joint_4_left','joint_5_left','joint_6_left']
        hello_str.position = [0,0,0,0,0,0,0,0,joint_0,joint_1,joint_2,joint_3,joint_4,joint_5]
        print(hello_str)
        hello_str.velocity = []
        hello_str.effort = []

        self.pub.publish(hello_str)
        ############################################



    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(405, 211)
        self.pushButton = QtGui.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(150, 170, 99, 27))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.label = QtGui.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(20, 30, 51, 17))
        self.label.setObjectName(_fromUtf8("label"))
        self.label_2 = QtGui.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(150, 30, 51, 17))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(Form)
        self.label_3.setGeometry(QtCore.QRect(280, 30, 51, 17))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(Form)
        self.label_4.setGeometry(QtCore.QRect(20, 90, 51, 17))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.label_5 = QtGui.QLabel(Form)
        self.label_5.setGeometry(QtCore.QRect(150, 90, 51, 17))
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.label_6 = QtGui.QLabel(Form)
        self.label_6.setGeometry(QtCore.QRect(280, 90, 51, 17))
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.textEdit = QtGui.QTextEdit(Form)
        self.textEdit.setGeometry(QtCore.QRect(20, 50, 104, 31))
        self.textEdit.setObjectName(_fromUtf8("textEdit"))
        self.textEdit_2 = QtGui.QTextEdit(Form)
        self.textEdit_2.setGeometry(QtCore.QRect(150, 50, 104, 31))
        self.textEdit_2.setObjectName(_fromUtf8("textEdit_2"))
        self.textEdit_3 = QtGui.QTextEdit(Form)
        self.textEdit_3.setGeometry(QtCore.QRect(280, 50, 104, 31))
        self.textEdit_3.setObjectName(_fromUtf8("textEdit_3"))
        self.textEdit_4 = QtGui.QTextEdit(Form)
        self.textEdit_4.setGeometry(QtCore.QRect(20, 120, 104, 31))
        self.textEdit_4.setObjectName(_fromUtf8("textEdit_4"))
        self.textEdit_5 = QtGui.QTextEdit(Form)
        self.textEdit_5.setGeometry(QtCore.QRect(150, 120, 104, 31))
        self.textEdit_5.setObjectName(_fromUtf8("textEdit_5"))
        self.textEdit_6 = QtGui.QTextEdit(Form)
        self.textEdit_6.setGeometry(QtCore.QRect(280, 120, 104, 31))
        self.textEdit_6.setObjectName(_fromUtf8("textEdit_6"))

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.pushButton, QtCore.SIGNAL(_fromUtf8("clicked()")), self.Get_angle)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "GUI_TOMO", None))
        self.pushButton.setText(_translate("Form", "Choose", None))
        self.label.setText(_translate("Form", "Joint 0", None))
        self.label_2.setText(_translate("Form", "Joint 1", None))
        self.label_3.setText(_translate("Form", "Joint 2", None))
        self.label_4.setText(_translate("Form", "Joint 3", None))
        self.label_5.setText(_translate("Form", "Joint 4", None))
        self.label_6.setText(_translate("Form", "Joint 5", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form(pub)
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

