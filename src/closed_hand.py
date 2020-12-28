import sys

sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")

import time
import Leap
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt


def main():
    controller = Leap.Controller()
    controller.config.set("tracking_processing_auto_flip", False)
    controller.config.save()
    closed_hand = False
    closed_hand_time = 0
    while 1:
        frame = controller.frame()
        if len(frame.hands):
            extended_finger_list = frame.fingers.extended()
            number_extended_fingers = len(extended_finger_list)
            if (
                number_extended_fingers == 0
            ):
                if closed_hand == False:
                    closed_hand_time = time.time()
                closed_hand = True
                if closed_hand == True and time.time() - closed_hand_time > 1:
                    print("Closed Hand")
                else:
                    print("Pending Closed Hand")
                # print(index_middle_tip_distance)
            else:
                print("No Closed Hand")
                closed_hand = False
                # print('Number of Extended Fingers: %s; Index: %s, Middle: %s' % (number_extended_fingers,index_extended,middle_extended))
            # print(len(extended_finger_list))


if __name__ == "__main__":
    main()