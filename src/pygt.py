#!/usr/bin/env python

import threading
import PyQt4.QtGui as qtw
from PyQt4 import QtSvg, QtCore
from PyQt4.QtCore import pyqtSlot
import subprocess
from PyQt4.QtGui import QFont, QIcon
import os
from env import set_env_var
from pynput.keyboard import Key, Controller
import rospy
from states import EdoStates
from util import Button, StateWrapper

set_env_var()
ip = os.getenv('IP')
port = int(os.getenv('PORT'))
height = int(os.getenv('HEIGHT'))
width = int(os.getenv('WIDTH'))
is_raspberry = bool(os.getenv('RASP'))
BASE_PATH = '/home/szymon/catkin_ws/src/calib/src/'


class Calibration_view(qtw.QWidget):
    def __init__(self):
        super(Calibration_view, self).__init__()
        self.joint_counter = 1

        layout = qtw.QGridLayout()

        svg_icon = add_svg(BASE_PATH + 'imgs/robotics.svg')

        self.label = qtw.QLabel("Joint number: {}".format(self.joint_counter))

        self.btn_next = add_button('CALIBRATE NEXT JOINT', self.fake_enter, 260)
        self.btn_next.pressed.connect(self.next_joint)
        self.btn_next.setStyleSheet('background-color: gold')
        shade_calib_btn = add_shadow()
        self.btn_next.setGraphicsEffect(shade_calib_btn)

        layout.addWidget(svg_icon, 0, 0, 1, 1, alignment=QtCore.Qt.AlignRight)
        layout.addWidget(self.label, 0, 1, 1, 2, alignment=QtCore.Qt.AlignLeft)
        layout.addWidget(self.btn_next, 0, 2, alignment=QtCore.Qt.AlignCenter)

        btn_right = add_button('>', self.arrow_right, 150, 100, BASE_PATH + 'imgs/arrow-right')
        btn_left = add_button('<', self.arrow_left, 150, 100, BASE_PATH + 'imgs/arrow-left')
        btn_up = add_button('^', self.arrow_up, 150, 100, BASE_PATH + 'imgs/arrow-up')
        btn_down = add_button('.', self.arrow_down, 150, 100, BASE_PATH + 'imgs/arrow-down')

        btn_up.pressed.connect(self.up_arrow_pressed)
        btn_up.released.connect(self.up_arrow_released)

        btn_right.pressed.connect(self.right_arrow_pressed)
        btn_right.released.connect(self.right_arrow_released)

        btn_left.pressed.connect(self.left_arrow_pressed)
        btn_left.released.connect(self.left_arrow_released)

        btn_down.pressed.connect(self.down_arrow_pressed)
        btn_down.released.connect(self.down_arrow_released)

        btn_break = add_button('UNBREAK', unbreak, 190, 100)
        btn_break.setStyleSheet('background-color: chocolate')
        shade_unbreak_btn = add_shadow()
        btn_break.setGraphicsEffect(shade_unbreak_btn)

        layout.addWidget(btn_left, 2, 0, alignment=QtCore.Qt.AlignRight)
        layout.addWidget(btn_break, 2, 1, alignment=QtCore.Qt.AlignCenter)
        layout.addWidget(btn_right, 2, 2, alignment=QtCore.Qt.AlignLeft)
        layout.addWidget(btn_up, 1, 1, alignment=QtCore.Qt.AlignCenter)
        layout.addWidget(btn_down, 3, 1, alignment=QtCore.Qt.AlignCenter)

        self.setLayout(layout)

        self.keyboard = Controller()

        self.command = StateWrapper(Button.NONE)
        print('started to calibrate')
        self.states = EdoStates(enable_algorithm_node=True, current_command=self.command)
        self.x = threading.Thread(target=self.calibrate_thread, args=(self.states,))
        self.x.daemon = True
        self.x.start()
        # subprocess.run(['powershell', '-Command', './scripts/calibration.ps1'])

    @pyqtSlot()
    def arrow_left(self):
        print('<')

    @pyqtSlot()
    def arrow_right(self):
        print('>')

    @pyqtSlot()
    def arrow_down(self):
        print('down')

    @pyqtSlot()
    def arrow_up(self):
        print('^')

    @pyqtSlot()
    def fake_enter(self):
        print('enter')
        self.command.state = Button.ENTER

        self.joint_counter += 1
        if self.joint_counter == 6:
            self.btn_next.setText('FINISH')
        if self.joint_counter > 6:
            self.joint_counter = 1
            # set text to default when closing
            self.btn_next.setText('CALIBRATE NEXT JOINT')
            self.close()
        self.label.setText("Joint number: {}".format(self.joint_counter))

    @pyqtSlot()
    def next_joint(self):
        pass

    @pyqtSlot()
    def up_arrow_pressed(self):
        self.command.state = Button.UP
        print("up pressed")

    @pyqtSlot()
    def up_arrow_released(self):
        self.command.state = Button.NONE
        print("up released")

    @pyqtSlot()
    def down_arrow_pressed(self):
        self.command.state = Button.DOWN
        print("down pressed")

    @pyqtSlot()
    def down_arrow_released(self):
        self.command.state = Button.NONE
        print("down released")

    @pyqtSlot()
    def right_arrow_pressed(self):
        self.command.state = Button.RIGHT
        print("right pressed")

    @pyqtSlot()
    def right_arrow_released(self):
        self.command.state = Button.NONE
        print("right released")

    @pyqtSlot()
    def left_arrow_pressed(self):
        self.command.state = Button.LEFT
        print("left pressed")

    @pyqtSlot()
    def left_arrow_released(self):
        self.command.state = Button.NONE
        print("left released")

    @pyqtSlot()
    def calibrate_thread(self, states):
        calibrated = False
        rospy.logwarn("Starting robot calibration procedure...")
        try:
            calibrated = states.calibration()
        except Exception as e:
            print(e)
            rospy.logerr("Robot calibration did not finish")
        else:
            if calibrated:
                rospy.logwarn("Robot is calibrated!")
            else:
                rospy.logerr("Robot calibration did not finish successfully")


class Display(qtw.QWidget):
    def __init__(self):
        super(Display, self).__init__()
        self.setWindowTitle('Controller')
        self.button_layout = qtw.QGridLayout()
        self.button_layout.setColumnStretch(0, 2)
        self.button_layout.setColumnStretch(1, 2)
        self.calibration_gui = None

        self.addUI()
        self.setLayout(self.button_layout)

        if is_raspberry:
            self.showFullScreen()
        else:
            self.setFixedSize(width, height)
            self.show()

    ### GENERAL UI ADDER
    def addUI(self):
        label = qtw.QLabel("STATUS: {}".format('robot status'))
        self.button_layout.addWidget(label, 0, 0, 1, 2, alignment=QtCore.Qt.AlignRight)

        svg_icon = add_svg(BASE_PATH + 'imgs/process.svg')
        self.button_layout.addWidget(svg_icon, 0, 0, 1, 2, alignment=QtCore.Qt.AlignCenter)

        btn1 = add_button('START', self.start)
        btn2 = add_button('STOP', self.stop)
        btn3 = add_button('UNBREAK', unbreak)
        btn4 = add_button('CALIBRATE', self.show_calibration)

        self.button_layout.addWidget(btn1, 1, 1)
        self.button_layout.addWidget(btn2, 1, 0)
        self.button_layout.addWidget(btn3, 2, 0)
        self.button_layout.addWidget(btn4, 2, 1)

    ### PARTICULAR WIDGET ADDERS
    def show_calibration(self):
        self.calibration_gui = Calibration_view()
        if is_raspberry:
            self.calibration_gui.showFullScreen()
        else:
            self.calibration_gui.setFixedSize(width, height)
            self.calibration_gui.show()

    def start(self):
        print('start')

    def stop(self):
        print('stop')


###COMMONS
def add_svg(path):
    svg_widget = QtSvg.QSvgWidget(path)
    svg_widget.setFixedSize(100, 100)
    return svg_widget


def unbreak(self):
    if not is_raspberry:
        print('unbreak')
    else:
        try:
            subprocess.call(BASE_PATH + 'scripts/unbreak.bash')
        except Exception as e:
            print(e)
            raise


def add_shadow():
    shadow = qtw.QGraphicsDropShadowEffect()
    shadow.setBlurRadius(20)
    shadow.setColor(QtCore.Qt.lightGray)
    return shadow


def add_button(name, func, wid=230, hei=130, icon_path=''):
    if icon_path:
        btn = qtw.QPushButton()
        btn.setIcon(QIcon(icon_path))
    else:
        btn = qtw.QPushButton(name)
    btn.setFixedSize(wid, hei)
    btn.clicked.connect(func)
    shade = add_shadow()
    btn.setGraphicsEffect(shade)
    return btn


def start():
    app = qtw.QApplication([])
    app.setFont(QFont('Microsoft YaHei Light', 13))
    mw = Display()
    mw.setFixedSize(width, height)
    if is_raspberry:
        subprocess.run(BASE_PATH + 'scripts/start.bash')
        mw.showFullScreen()
    else:
        mw.show()

    with open(BASE_PATH + 'style.qss', 'r') as f:
        _style = f.read()
        app.setStyleSheet(_style)

    app.setStyle(qtw.QStyleFactory.create('Fusion'))
    app.exec_()


if __name__ == '__main__':
    rospy.init_node('edo_calibrate', anonymous=True)
    start()
