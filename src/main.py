#!/usr/bin/env python

import threading
import rospy
from getkey import getkey, keys
from pynput.keyboard import Key, Controller
import time
from states import EdoStates


def calibrate_thread(states):

    calibrated = False
    rospy.logwarn("Starting robot calibration procedure...")
    try:
        calibrated = states.calibration()
    except:
        rospy.logerr("Robot calibration did not finish")
    else:
        if calibrated:
            rospy.logwarn("Robot is calibrated!")
        else:
            rospy.logerr("Robot calibration did not finish successfully")


def main():
    states = EdoStates(enable_algorithm_node=True)

    x = threading.Thread(target=calibrate_thread, args=(states, ))
    x.start()
    
    while True: # not states.CS_CALIBRATED:
        # DO CALIBRATION ACTIONS HERE
        time.sleep(5)
        print("################ " + str(states.get_current_code_string()))
        print("################ " + str(states.current_joint))

    x.join()


if __name__ == '__main__':
    rospy.init_node('edo_calibrate', anonymous=True)
    main()
