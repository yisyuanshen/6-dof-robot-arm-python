from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap, QFont
from PyQt5 import QtCore, QtGui, uic
import sys
import time
import json
import numpy as np

import kinematics
import plot_robot

class myGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('6 DoF Robot Arm Simulator')

        # set parameters
        self.win_w = 0
        self.win_h = 0

        # title box
        self.title = QLabel('6 DoF Robot Arm Simulation Tool', self)
        self.title.setFont(QFont('Comic Sans MS', 20, QFont.Bold))
        self.title.setStyleSheet('color: white; background-color: black')
        self.title.setAlignment(QtCore.Qt.AlignCenter)

        self.hbox_title = QHBoxLayout()
        self.hbox_title.stretch(1)
        self.hbox_title.addWidget(self.title)

        # mode box
        self.mode_setting = QPushButton('Robot Arm Setting', self)
        self.mode_setting.setFont(QFont('Comic Sans MS', 12, QFont.Medium))
        self.mode_setting.setStyleSheet('background-color: lightpink;')
        self.mode_setting.clicked.connect(self.robot_setting)

        self.mode_forward = QPushButton('Forward Kinematics', self)
        self.mode_forward.setFont(QFont('Comic Sans MS', 12, QFont.Medium))
        self.mode_forward.setStyleSheet('background-color: skyblue;')
        self.mode_forward.clicked.connect(self.forward_kinematics)

        self.mode_inverse = QPushButton('Inverse Kinematics', self)
        self.mode_inverse.setFont(QFont('Comic Sans MS', 12, QFont.Medium))
        self.mode_inverse.setStyleSheet('background-color: lightgreen;')
        self.mode_inverse.clicked.connect(self.inverse_kinematics)

        self.hbox_mode = QHBoxLayout()
        self.hbox_mode.stretch(1)
        self.hbox_mode.addWidget(self.mode_setting)
        self.hbox_mode.addWidget(self.mode_forward)
        self.hbox_mode.addWidget(self.mode_inverse)

        # robot arm setting (DH-Table, func_btn, robot plot)
        self.title_dh_table = QLabel('Denavit - Hartenberg Table', self)
        self.title_dh_table.setFont(QFont('Comic Sans MS', 14, QFont.Medium))
        self.title_dh_table.setStyleSheet('color: black')
        self.title_dh_table.setAlignment(QtCore.Qt.AlignCenter)

        self.hbox_title_dh_table = QHBoxLayout()
        self.hbox_title_dh_table.stretch(1)
        self.hbox_title_dh_table.addWidget(self.title_dh_table)

        self.label_a = [QLabel('a1:',self),QLabel('a2:',self),QLabel('a3:',self),
                        QLabel('a4:',self),QLabel('a5:',self),QLabel('a6:',self), ]
        self.text_a = [QLineEdit('',self),QLineEdit('',self),QLineEdit('',self),
                       QLineEdit('',self),QLineEdit('',self),QLineEdit('',self),]
        self.label_alpha = [QLabel(' alpha1:',self),QLabel(' alpha2:',self),QLabel(' alpha3:',self),
                        QLabel(' alpha4:',self),QLabel(' alpha5:',self),QLabel(' alpha6:',self), ]
        self.text_alpha = [QLineEdit('',self),QLineEdit('',self),QLineEdit('',self),
                       QLineEdit('',self),QLineEdit('',self),QLineEdit('',self),]
        self.label_theta = [QLabel(' theta1:',self),QLabel(' theta2:',self),QLabel(' theta3:',self),
                        QLabel(' theta4:',self),QLabel(' theta5:',self),QLabel(' theta6:',self), ]
        self.text_theta = [QLineEdit('',self),QLineEdit('',self),QLineEdit('',self),
                       QLineEdit('',self),QLineEdit('',self),QLineEdit('',self),]
        self.label_d = [QLabel(' d1:',self),QLabel(' d2:',self),QLabel(' d3:',self),
                        QLabel(' d4:',self),QLabel(' d5:',self),QLabel(' d6:',self), ]
        self.text_d = [QLineEdit('',self),QLineEdit('',self),QLineEdit('',self),
                       QLineEdit('',self),QLineEdit('',self),QLineEdit('',self),]
        
        self.dh_param = [self.label_a, self.text_a, self.label_alpha, self.text_alpha,
                             self.label_theta, self.text_theta, self.label_d, self.text_d]
        
        self.hbox_dh_param = [QHBoxLayout(), QHBoxLayout(), QHBoxLayout(), 
                         QHBoxLayout(), QHBoxLayout(), QHBoxLayout()]
        
        for i in range(8):
            for j in range(6):
                self.dh_param[i][j].setFont(QFont('Comic Sans MS', 10, QFont.Medium))
                if i%2 == 0: self.dh_param[i][j].setStyleSheet('color: black')
                else: self.dh_param[i][j].setStyleSheet('color: black; background-color: white')
                self.dh_param[i][j].setAlignment(QtCore.Qt.AlignCenter)
                self.hbox_dh_param[j].stretch(1)
                self.hbox_dh_param[j].addWidget(self.dh_param[i][j])

        self.btn_import = QPushButton('Import', self)
        self.btn_import.setFont(QFont('Comic Sans MS', 12, QFont.Medium))
        self.btn_import.setStyleSheet('background-color: lightcyan;')
        self.btn_import.clicked.connect(self.import_data)

        self.btn_export = QPushButton('Export', self)
        self.btn_export.setFont(QFont('Comic Sans MS', 12, QFont.Medium))
        self.btn_export.setStyleSheet('background-color: lightcyan;')
        self.btn_export.clicked.connect(self.export_data)

        self.btn_apply = QPushButton('Apply', self)
        self.btn_apply.setFont(QFont('Comic Sans MS', 12, QFont.Medium))
        self.btn_apply.setStyleSheet('background-color: lightcyan;')
        self.btn_apply.clicked.connect(self.apply_data)

        self.hbox_btn_func = QHBoxLayout()
        self.hbox_btn_func.stretch(1)
        self.hbox_btn_func.addWidget(self.btn_import)
        self.hbox_btn_func.addWidget(self.btn_export)
        self.hbox_btn_func.addWidget(self.btn_apply)

        self.plot_robot = QLabel(self)
        self.pixmap = QPixmap('img.jpg').scaled(400, 400)
        self.plot_robot.setPixmap(self.pixmap)
        self.plot_robot.resize(self.pixmap.width(), self.pixmap.height())

        self.hbox_plot_robot = QHBoxLayout()
        self.hbox_plot_robot.stretch(1)
        self.hbox_plot_robot.addWidget(self.plot_robot)

        # forward kinematics
        self.forward_info = QLabel()
        self.forward_info.setText('Forward Kinematics Info...')
        self.forward_info.setFont(QFont('Comic Sans MS', 12, QFont.Medium))
        self.hbox_forward_info = QHBoxLayout()
        self.hbox_forward_info.addWidget(self.forward_info)

        self.forward_camera = [QSlider(), QSlider(), QSlider()]
        self.joint = [QSlider(), QSlider(), QSlider(),
                      QSlider(), QSlider(), QSlider()]

        self.vbox_slider = QVBoxLayout()
        for i in range(3): 
            self.forward_camera[i].setOrientation(1)
            self.forward_camera[i].setStyleSheet('''
                QSlider {
                    border-radius: 10px;
                }
                QSlider::groove:horizontal {
                    height: 5px;
                    background: #000;
                }
                QSlider::handle:horizontal{
                    background: #f00;
                    width: 16px;
                    height: 16px;
                    margin:-6px 0;
                    border-radius:8px;
                }
                QSlider::sub-page:horizontal{
                    background:#ff0;
                }
            ''')
            self.forward_camera[i].setMinimum(0)
            self.forward_camera[i].setMaximum(100)
            self.forward_camera[i].setValue(50)
            self.forward_camera[i].valueChanged.connect(self.apply_angle)
            self.vbox_slider.addWidget(self.forward_camera[i])
        for i in range(6):
            self.joint[i].setOrientation(1)
            self.joint[i].setStyleSheet('''
                QSlider {
                    border-radius: 10px;
                }
                QSlider::groove:horizontal {
                    height: 5px;
                    background: #000;
                }
                QSlider::handle:horizontal{
                    background: #f00;
                    width: 16px;
                    height: 16px;
                    margin:-6px 0;
                    border-radius:8px;
                }
                QSlider::sub-page:horizontal{
                    background:#ff0;
                }
            ''')
            self.joint[i].setMinimum(-180)
            self.joint[i].setMaximum(180)
            self.joint[i].setValue(0)
            self.joint[i].valueChanged.connect(self.apply_angle)
            self.vbox_slider.addWidget(self.joint[i])

        self.forward_label_slider = [QLabel('Camera X'), QLabel('Camara Y'), QLabel('Camera Z'),
                             QLabel('Joint 1'), QLabel('Joint 2'), QLabel('Joint 3'),
                             QLabel('Joint 4'), QLabel('Joint 5'), QLabel('Joint 6')]
        self.forward_value_slider = [QLabel(str(self.forward_camera[0].value())), QLabel(str(self.forward_camera[1].value())), QLabel(str(self.forward_camera[2].value())),
                             QLabel(str(self.joint[0].value())), QLabel(str(self.joint[1].value())), QLabel(str(self.joint[2].value())),
                             QLabel(str(self.joint[3].value())), QLabel(str(self.joint[4].value())), QLabel(str(self.joint[5].value()))]

        self.vbox_forward_label_slider = QVBoxLayout()
        self.vbox_forward_value_slider = QVBoxLayout()
        
        for i in range(9): 
            self.forward_label_slider[i].setFont(QFont('Comic Sans MS', 8, QFont.Medium))
            self.vbox_forward_label_slider.addWidget(self.forward_label_slider[i])
            self.forward_value_slider[i].setFont(QFont('Comic Sans MS', 8, QFont.Medium))
            self.vbox_forward_value_slider.addWidget(self.forward_value_slider[i])

        # inverse kinematics
        self.inverse_info = QLabel()
        self.inverse_info.setText('Inverse Kinematics Info...')
        self.inverse_info.setFont(QFont('Comic Sans MS', 12, QFont.Medium))
        self.hbox_inverse_info = QHBoxLayout()
        self.hbox_inverse_info.addWidget(self.inverse_info)

        self.inverse_camera = [QSlider(), QSlider(), QSlider()]
        self.position = [QSlider(), QSlider(), QSlider(),
                      QSlider(), QSlider(), QSlider()]

        self.vbox_inverse_slider = QVBoxLayout()
        for i in range(3): 
            self.inverse_camera[i].setOrientation(1)
            self.inverse_camera[i].setStyleSheet('''
                QSlider {
                    border-radius: 10px;
                }
                QSlider::groove:horizontal {
                    height: 5px;
                    background: #000;
                }
                QSlider::handle:horizontal{
                    background: #f00;
                    width: 16px;
                    height: 16px;
                    margin:-6px 0;
                    border-radius:8px;
                }
                QSlider::sub-page:horizontal{
                    background:#ff0;
                }
            ''')
            self.inverse_camera[i].setMinimum(0)
            self.inverse_camera[i].setMaximum(100)
            self.inverse_camera[i].setValue(50)
            self.inverse_camera[i].valueChanged.connect(self.apply_pos)
            self.vbox_slider.addWidget(self.inverse_camera[i])
        for i in range(6):
            self.position[i].setOrientation(1)
            self.position[i].setStyleSheet('''
                QSlider {
                    border-radius: 10px;
                }
                QSlider::groove:horizontal {
                    height: 5px;
                    background: #000;
                }
                QSlider::handle:horizontal{
                    background: #f00;
                    width: 16px;
                    height: 16px;
                    margin:-6px 0;
                    border-radius:8px;
                }
                QSlider::sub-page:horizontal{
                    background:#ff0;
                }
            ''')
            self.position[i].setMinimum(-180)
            self.position[i].setMaximum(180)
            self.position[i].setValue(0)
            self.position[i].valueChanged.connect(self.apply_pos)
            self.vbox_slider.addWidget(self.position[i])

        self.inverse_label_slider = [QLabel('Camera X'), QLabel('Camara Y'), QLabel('Camera Z'),
                                     QLabel('Robot X'), QLabel('Robot Y'), QLabel('Robot Z'),
                                     QLabel('Robot Rx'), QLabel('Robot Ry'), QLabel('Robot Rz')]
        self.inverse_value_slider = [QLabel(str(self.inverse_camera[0].value())), QLabel(str(self.inverse_camera[1].value())), QLabel(str(self.inverse_camera[2].value())),
                                     QLabel(str(self.position[0].value())), QLabel(str(self.position[1].value())), QLabel(str(self.position[2].value())),
                                     QLabel(str(self.position[3].value())), QLabel(str(self.position[4].value())), QLabel(str(self.position[5].value()))]

        self.vbox_inverse_label_slider = QVBoxLayout()
        self.vbox_inverse_value_slider = QVBoxLayout()
        
        for i in range(9): 
            self.inverse_label_slider[i].setFont(QFont('Comic Sans MS', 8, QFont.Medium))
            self.vbox_inverse_label_slider.addWidget(self.inverse_label_slider[i])
            self.inverse_value_slider[i].setFont(QFont('Comic Sans MS', 8, QFont.Medium))
            self.vbox_inverse_value_slider.addWidget(self.inverse_value_slider[i])

        # layout
        self.gbox = QGridLayout()
        self.gbox.addItem(self.hbox_title, 0, 0, 1, 15)
        self.gbox.addItem(self.hbox_mode, 1, 0, 1, 15)
        self.gbox.addItem(self.hbox_plot_robot, 2, 10, 11, 5)

        # Setting
        self.gbox.addItem(self.hbox_title_dh_table, 3, 0, 1, 10)
        for i in range(6): self.gbox.addItem(self.hbox_dh_param[i],4+i, 0, 1, 10)
        self.gbox.addItem(self.hbox_btn_func, 11, 0, 1, 10)

        # forward
        self.gbox.addItem(self.hbox_forward_info, 2, 0, 2, 9)
        self.gbox.addItem(self.vbox_forward_label_slider, 4, 0, 9, 1)
        self.gbox.addItem(self.vbox_slider, 4, 1, 9, 7)
        self.gbox.addItem(self.vbox_forward_value_slider, 4, 8, 9, 1)
        self.forward_info.hide()
        for i in range(9): 
            self.forward_label_slider[i].hide()
            self.forward_value_slider[i].hide()
        for i in range(3): self.forward_camera[i].hide()
        for i in range(6): self.joint[i].hide()

        # inverse
        self.gbox.addItem(self.hbox_inverse_info, 2, 0, 2, 9)
        self.gbox.addItem(self.vbox_inverse_label_slider, 4, 0, 9, 1)
        self.gbox.addItem(self.vbox_slider, 4, 1, 9, 7)
        self.gbox.addItem(self.vbox_inverse_value_slider, 4, 8, 9, 1)
        self.inverse_info.hide()
        for i in range(9): 
            self.inverse_label_slider[i].hide()
            self.inverse_value_slider[i].hide()
        for i in range(3): self.inverse_camera[i].hide()
        for i in range(6): self.position[i].hide()


        self.setLayout(self.gbox)

        # timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000)

        # create the window
        self.setStyleSheet("background-color: Lightgray;")
        self.setGeometry(200, 200, self.win_w, self.win_h)
        self.show()

    def robot_setting(self,):
        print('\nButton: robot setting\n')
        for i in range(len(self.dh_param)):
            for j in range(len(self.dh_param[i])):
                self.dh_param[i][j].show()
        self.title_dh_table.show()
        self.btn_import.show()
        self.btn_export.show()
        self.btn_apply.show()

        self.forward_info.hide()
        for i in range(9): 
            self.forward_label_slider[i].hide()
            self.forward_value_slider[i].hide()
        for i in range(3): self.forward_camera[i].hide()
        for i in range(6): self.joint[i].hide()

        self.inverse_info.hide()
        for i in range(9): 
            self.inverse_label_slider[i].hide()
            self.inverse_value_slider[i].hide()
        for i in range(3): self.inverse_camera[i].hide()
        for i in range(6): self.position[i].hide()
    
    def forward_kinematics(self,):
        print('\nButton: forward kinematics\n')
        for i in range(len(self.dh_param)):
            for j in range(len(self.dh_param[i])):
                self.dh_param[i][j].hide()
        self.title_dh_table.hide()
        self.btn_import.hide()
        self.btn_export.hide()
        self.btn_apply.hide()

        self.inverse_info.hide()
        for i in range(9): 
            self.inverse_label_slider[i].hide()
            self.inverse_value_slider[i].hide()
        for i in range(3): self.inverse_camera[i].hide()
        for i in range(6): self.position[i].hide()

        self.forward_info.show()
        for i in range(9): 
            self.forward_label_slider[i].show()
            self.forward_value_slider[i].show()
        for i in range(3): self.forward_camera[i].show()
        for i in range(6): self.joint[i].show()


    def inverse_kinematics(self,):
        print('\nButton: inverse kinematics\n')
        for i in range(len(self.dh_param)):
            for j in range(len(self.dh_param[i])):
                self.dh_param[i][j].hide()
        self.title_dh_table.hide()
        self.btn_import.hide()
        self.btn_export.hide()
        self.btn_apply.hide()

        self.forward_info.hide()
        for i in range(9): 
            self.forward_label_slider[i].hide()
            self.forward_value_slider[i].hide()
        for i in range(3): self.forward_camera[i].hide()
        for i in range(6): self.joint[i].hide()

        self.inverse_info.show()
        for i in range(9): 
            self.inverse_label_slider[i].show()
            self.inverse_value_slider[i].show()
        for i in range(3): self.inverse_camera[i].show()
        for i in range(6): self.position[i].show()

    def import_data(self,):
        print('\nButton: import data\n')
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        filename, _ = QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","JSON Files (*.json)", options=options)
        if filename:
            jsonfile = open(str(filename))
        data = json.load(jsonfile)

        for i in range(6):
           self.text_a[i].setText(data['Joint'+str(i+1)]['a'])
           self.text_alpha[i].setText(data['Joint'+str(i+1)]['alpha'])
           self.text_theta[i].setText(data['Joint'+str(i+1)]['theta'])
           self.text_d[i].setText(data['Joint'+str(i+1)]['d'])
           
    def export_data(self,):
        print('\nButton: export data\n')
        data = {'Joint1':{}, 'Joint2':{}, 'Joint3':{}, 
                'Joint4':{}, 'Joint5':{}, 'Joint6':{}}
        for i in range(6):
            data['Joint'+str(i+1)]['a'] = self.text_a[i].text()
            data['Joint'+str(i+1)]['alpha'] = self.text_alpha[i].text()
            data['Joint'+str(i+1)]['theta'] = self.text_theta[i].text()
            data['Joint'+str(i+1)]['d'] = self.text_d[i].text()

        filename = QFileDialog.getSaveFileName(self, 'Save File', "robot_config", "JSON (*.json)")
        
        if '.json' not in filename[0]:
            if filename[0] == '': return
            filename = filename[0] + '.json'
        else:
            filename = filename[0]
        with open(filename, 'w') as jsonfile:
            json.dump(data, jsonfile, indent=3)
            jsonfile.close()   
        
    def apply_data(self,):
        
        print('\nButton: apply data\n')
        forward_T = [] # [T01, T12, ...]
        pos = [np.matrix([0,0,0,1]).T]
        for i in range(6):
            forward_T.append(kinematics.forward_T(float(self.text_a[i].text()), 
                             float(self.text_alpha[i].text()), float(self.text_theta[i].text()), 
                             float(self.text_d[i].text())))
            T = forward_T[0]
            for j in range(len(pos)-1):
                T = T*forward_T[j+1]
            pos.append(T*pos[0])

        self.img = plot_robot.plot(pos)
        height, width, channel = self.img.shape
        bytesPerline = 3 * width
        self.qImg = QtGui.QImage(self.img.data, width, height, bytesPerline, QtGui.QImage.Format_RGB888).rgbSwapped()
        self.plot_robot.setPixmap(QPixmap.fromImage(self.qImg))


    def apply_angle(self,):
        for i in range(3):
            self.forward_value_slider[i].setText(str(self.forward_camera[i].value()))
        for i in range(6):
            self.forward_value_slider[i+3].setText(str(self.joint[i].value()))

        forward_T = [] # [T01, T12, ...]
        pos = [np.matrix([0,0,0,1]).T]
        for i in range(6):
            forward_T.append(kinematics.forward_T(float(self.text_a[i].text()), 
                             float(self.text_alpha[i].text()), float(self.text_theta[i].text())+self.joint[i].value(), 
                             float(self.text_d[i].text())))
            T = forward_T[0]
            for j in range(len(pos)-1):
                T = T*forward_T[j+1]
            pos.append(T*pos[0])

        self.img = plot_robot.plot(pos)
        height, width, channel = self.img.shape
        bytesPerline = 3 * width
        self.qImg = QtGui.QImage(self.img.data, width, height, bytesPerline, QtGui.QImage.Format_RGB888).rgbSwapped()
        self.plot_robot.setPixmap(QPixmap.fromImage(self.qImg))

    def apply_pos(self,):
        for i in range(3):
            self.inverse_value_slider[i].setText(str(self.inverse_camera[i].value()))
        for i in range(6):
            self.inverse_value_slider[i+3].setText(str(self.position[i].value()))

    def update(self):
        print("[{}] current time...".format(round(time.time(), 5)))
        pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = myGUI()
    sys.exit(app.exec_())