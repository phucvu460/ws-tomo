# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GUI_2.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

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
    def Get_angle(self):
        print("Choose button is already clicked")
        
        joint_0 = float(self.textEdit.toPlainText())
        joint_1 = float(self.textEdit_2.toPlainText())
        joint_2 = float(self.textEdit_3.toPlainText())
        joint_3 = float(self.textEdit_4.toPlainText())
        joint_4 = float(self.textEdit_5.toPlainText())
        joint_5 = float(self.textEdit_6.toPlainText())


        self.horizontalSlider.setSliderPosition(joint_0*100)
        self.horizontalSlider_2.setSliderPosition(joint_1*100)
        self.horizontalSlider_3.setSliderPosition(joint_2*100)
        self.horizontalSlider_4.setSliderPosition(joint_3*100)
        self.horizontalSlider_5.setSliderPosition(joint_4*100)
        self.horizontalSlider_6.setSliderPosition(joint_5*100)

        print(joint_1 * joint_2)
        print(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)


    def Mode_Slider(self):
        scale_value0 = float(self.horizontalSlider.value())/100
        scale_value1 = float(self.horizontalSlider_2.value())/100
        scale_value2 = float(self.horizontalSlider_3.value())/100
        scale_value3 = float(self.horizontalSlider_4.value())/100
        scale_value4 = float(self.horizontalSlider_5.value())/100
        scale_value5 = float(self.horizontalSlider_6.value())/100
        print(scale_value0,scale_value1,scale_value2,scale_value3,scale_value4,scale_value5)

        # self.doubleSpinBox.setValue(scale_value0)
        # self.doubleSpinBox_2.setValue(scale_value1)
        # self.doubleSpinBox_3.setValue(scale_value2)
        # self.doubleSpinBox_4.setValue(scale_value3)
        # self.doubleSpinBox_5.setValue(scale_value4)
        # self.doubleSpinBox_6.setValue(scale_value5)
            
    def update_spinbox(self, value):
        self.doubleSpinBox.setValue(float(value)/100)
    def update_spinbox1(self, value):
        self.doubleSpinBox_2.setValue(float(value)/100)
    def update_spinbox2(self, value):
        self.doubleSpinBox_3.setValue(float(value)/100)
    def update_spinbox3(self, value):
        self.doubleSpinBox_4.setValue(float(value)/100)
    def update_spinbox4(self, value):
        self.doubleSpinBox_5.setValue(float(value)/100)
    def update_spinbox5(self, value):
        self.doubleSpinBox_6.setValue(float(value)/100)

    # def update_slider_position(self):
    #     self.horizontalSlider.setSliderPosition(314)
    #     self.horizontalSlider.setSliderPosition(self.doubleSpinBox_2.value()*100)
    #     self.horizontalSlider.setSliderPosition(self.doubleSpinBox_3.value()*100)
    #     self.horizontalSlider.setSliderPosition(self.doubleSpinBox_4.value()*100)
    #     self.horizontalSlider.setSliderPosition(self.doubleSpinBox_5.value()*100)
    #     self.horizontalSlider.setSliderPosition(self.doubleSpinBox_6.value()*100)

    # def valueHandler(self,value):   
    #     scaledValue = float(value)/100     #type of "value" is int so you need to convert it to float in order to get float type for "scaledValue" 
    #     print scaledValue , type(scaledValue)
    #     self.spinBox.setValue(float(self.horizontalSlider.value())/100)

    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(521, 715)
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
        self.textEdit_3.setAutoFillBackground(False)
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
        self.label_7 = QtGui.QLabel(Form)
        self.label_7.setGeometry(QtCore.QRect(180, 240, 51, 17))
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.label_8 = QtGui.QLabel(Form)
        self.label_8.setGeometry(QtCore.QRect(180, 310, 51, 17))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.label_9 = QtGui.QLabel(Form)
        self.label_9.setGeometry(QtCore.QRect(180, 380, 51, 17))
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.label_10 = QtGui.QLabel(Form)
        self.label_10.setGeometry(QtCore.QRect(180, 450, 51, 17))
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.label_11 = QtGui.QLabel(Form)
        self.label_11.setGeometry(QtCore.QRect(180, 520, 51, 17))
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.label_12 = QtGui.QLabel(Form)
        self.label_12.setGeometry(QtCore.QRect(180, 590, 51, 17))
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.pushButton_2 = QtGui.QPushButton(Form)
        self.pushButton_2.setGeometry(QtCore.QRect(130, 670, 141, 31))
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalSlider = QtGui.QSlider(Form)
        self.horizontalSlider.setGeometry(QtCore.QRect(50, 250, 311, 61))
        self.horizontalSlider.setMinimum(-314)
        self.horizontalSlider.setMaximum(314)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName(_fromUtf8("horizontalSlider"))
        self.horizontalSlider_2 = QtGui.QSlider(Form)
        self.horizontalSlider_2.setGeometry(QtCore.QRect(50, 330, 311, 61))
        self.horizontalSlider_2.setMinimum(-314)
        self.horizontalSlider_2.setMaximum(314)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName(_fromUtf8("horizontalSlider_2"))
        self.horizontalSlider_3 = QtGui.QSlider(Form)
        self.horizontalSlider_3.setGeometry(QtCore.QRect(50, 400, 311, 61))
        self.horizontalSlider_3.setMinimum(-314)
        self.horizontalSlider_3.setMaximum(314)
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setObjectName(_fromUtf8("horizontalSlider_3"))
        self.horizontalSlider_4 = QtGui.QSlider(Form)
        self.horizontalSlider_4.setGeometry(QtCore.QRect(50, 460, 311, 61))
        self.horizontalSlider_4.setMinimum(-314)
        self.horizontalSlider_4.setMaximum(314)
        self.horizontalSlider_4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_4.setObjectName(_fromUtf8("horizontalSlider_4"))
        self.horizontalSlider_5 = QtGui.QSlider(Form)
        self.horizontalSlider_5.setGeometry(QtCore.QRect(50, 540, 311, 61))
        self.horizontalSlider_5.setMinimum(-314)
        self.horizontalSlider_5.setMaximum(314)
        self.horizontalSlider_5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_5.setObjectName(_fromUtf8("horizontalSlider_5"))
        self.horizontalSlider_6 = QtGui.QSlider(Form)
        self.horizontalSlider_6.setGeometry(QtCore.QRect(50, 610, 311, 61))
        self.horizontalSlider_6.setMinimum(-314)
        self.horizontalSlider_6.setMaximum(314)
        self.horizontalSlider_6.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_6.setObjectName(_fromUtf8("horizontalSlider_6"))
        self.doubleSpinBox = QtGui.QDoubleSpinBox(Form)
        self.doubleSpinBox.setGeometry(QtCore.QRect(290, 240, 62, 27))
        self.doubleSpinBox.setDecimals(2)
        self.doubleSpinBox.setMinimum(-3.14)
        self.doubleSpinBox.setMaximum(3.14)
        self.doubleSpinBox.setSingleStep(0.01)
        self.doubleSpinBox.setObjectName(_fromUtf8("doubleSpinBox"))
        self.doubleSpinBox_2 = QtGui.QDoubleSpinBox(Form)
        self.doubleSpinBox_2.setGeometry(QtCore.QRect(290, 320, 62, 27))
        self.doubleSpinBox_2.setDecimals(2)
        self.doubleSpinBox_2.setMinimum(-3.14)
        self.doubleSpinBox_2.setMaximum(3.14)
        self.doubleSpinBox_2.setSingleStep(0.01)
        self.doubleSpinBox_2.setObjectName(_fromUtf8("doubleSpinBox_2"))
        self.doubleSpinBox_3 = QtGui.QDoubleSpinBox(Form)
        self.doubleSpinBox_3.setGeometry(QtCore.QRect(290, 390, 62, 27))
        self.doubleSpinBox_3.setDecimals(2)
        self.doubleSpinBox_3.setMinimum(-3.14)
        self.doubleSpinBox_3.setMaximum(3.14)
        self.doubleSpinBox_3.setSingleStep(0.01)
        self.doubleSpinBox_3.setObjectName(_fromUtf8("doubleSpinBox_3"))
        self.doubleSpinBox_4 = QtGui.QDoubleSpinBox(Form)
        self.doubleSpinBox_4.setGeometry(QtCore.QRect(290, 450, 62, 27))
        self.doubleSpinBox_4.setDecimals(2)
        self.doubleSpinBox_4.setMinimum(-3.14)
        self.doubleSpinBox_4.setMaximum(3.14)
        self.doubleSpinBox_4.setSingleStep(0.01)
        self.doubleSpinBox_4.setObjectName(_fromUtf8("doubleSpinBox_4"))
        self.doubleSpinBox_5 = QtGui.QDoubleSpinBox(Form)
        self.doubleSpinBox_5.setGeometry(QtCore.QRect(290, 530, 62, 27))
        self.doubleSpinBox_5.setDecimals(2)
        self.doubleSpinBox_5.setMinimum(-3.14)
        self.doubleSpinBox_5.setMaximum(3.14)
        self.doubleSpinBox_5.setSingleStep(0.01)
        self.doubleSpinBox_5.setObjectName(_fromUtf8("doubleSpinBox_5"))
        self.doubleSpinBox_6 = QtGui.QDoubleSpinBox(Form)
        self.doubleSpinBox_6.setGeometry(QtCore.QRect(290, 600, 62, 27))
        self.doubleSpinBox_6.setDecimals(2)
        self.doubleSpinBox_6.setMinimum(-3.14)
        self.doubleSpinBox_6.setMaximum(3.14)
        self.doubleSpinBox_6.setSingleStep(0.01)
        self.doubleSpinBox_6.setObjectName(_fromUtf8("doubleSpinBox_6"))

        self.retranslateUi(Form)

        self.horizontalSlider.valueChanged[int].connect(self.update_spinbox)
        
        # self.doubleSpinBox.editingFinished.connect(self.update_slider_position)
        self.horizontalSlider_2.valueChanged[int].connect(self.update_spinbox1)
        # self.doubleSpinBox_2.editingFinished.connect(self.update_slider_position)
        self.horizontalSlider_3.valueChanged[int].connect(self.update_spinbox2)
        # self.doubleSpinBox_3.editingFinished.connect(self.update_slider_position)
        self.horizontalSlider_4.valueChanged[int].connect(self.update_spinbox3)
        # self.doubleSpinBox_4.editingFinished.connect(self.update_slider_position)
        self.horizontalSlider_5.valueChanged[int].connect(self.update_spinbox4)
        # self.doubleSpinBox_5.editingFinished.connect(self.update_slider_position)
        self.horizontalSlider_6.valueChanged[int].connect(self.update_spinbox5)
        # self.doubleSpinBox_6.editingFinished.connect(self.update_slider_position)
        
        QtCore.QObject.connect(self.pushButton, QtCore.SIGNAL(_fromUtf8("clicked()")), self.Get_angle)
        QtCore.QObject.connect(self.pushButton_2, QtCore.SIGNAL(_fromUtf8("clicked()")), self.Mode_Slider)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.pushButton.setText(_translate("Form", "Choose", None))
        self.label.setText(_translate("Form", "Joint 0", None))
        self.label_2.setText(_translate("Form", "Joint 1", None))
        self.label_3.setText(_translate("Form", "Joint 2", None))
        self.label_4.setText(_translate("Form", "Joint 3", None))
        self.label_5.setText(_translate("Form", "Joint 4", None))
        self.label_6.setText(_translate("Form", "Joint 5", None))
        self.label_7.setText(_translate("Form", "Joint 0", None))
        self.label_8.setText(_translate("Form", "Joint 1", None))
        self.label_9.setText(_translate("Form", "Joint 2", None))
        self.label_10.setText(_translate("Form", "Joint 3", None))
        self.label_11.setText(_translate("Form", "Joint 4", None))
        self.label_12.setText(_translate("Form", "Joint 5", None))
        self.pushButton_2.setText(_translate("Form", "Choose Slider", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

