#!/usr/bin/env python

import threading
import rospy
from getkey import getkey, keys
from pynput.keyboard import Key, Controller
from states import EdoStates


def test():
    key = getkey()
    if key == keys.ENTER:
        print("ENTER")
    else:
        print("Other key")


def calibrate_thread():
    test()

    # states = EdoStates(enable_algorithm_node=True)
    # calibrated = False
    # rospy.logwarn("Starting robot calibration procedure...")
    # try:
    #     calibrated = states.calibration()
    # except:
    #     rospy.logerr("Robot calibration did not finish")
    # else:
    #     if calibrated:
    #         rospy.logwarn("Robot is calibrated!")
    #     else:
    #         rospy.logerr("Robot calibration did not finish successfully")


def main():
    x = threading.Thread(target=calibrate_thread)
    x.start()
    keyboard = Controller()
    keyboard.press('a')
    keyboard.release('a')


if __name__ == '__main__':
    rospy.init_node('edo_calibrate', anonymous=True)
    main()
