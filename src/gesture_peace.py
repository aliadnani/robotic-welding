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
    peace_extended = False
    peace_extended_time = 0
    while 1:
        frame = controller.frame()
        if len(frame.hands):
            extended_finger_list = frame.fingers.extended()
            index_extended = False
            middle_extended = False
            number_extended_fingers = len(extended_finger_list)

            # Calculate distance from index to middle
            index_finger_tip_pos = (
                frame.hands[0]
                .fingers.finger_type(Leap.Finger.TYPE_INDEX)[0]
                .bone(3)
                .next_joint.to_tuple()
            )
            middle_finger_tip_pos = (
                frame.hands[0]
                .fingers.finger_type(Leap.Finger.TYPE_MIDDLE)[0]
                .bone(3)
                .next_joint.to_tuple()
            )
            index_middle_tip_distance = np.linalg.norm(
                np.array(index_finger_tip_pos) - np.array(middle_finger_tip_pos)
            )

            # Check extendedness of fingers
            for finger in extended_finger_list:
                if finger.type == Leap.Finger.TYPE_INDEX:
                    index_extended = True
                elif finger.type == Leap.Finger.TYPE_MIDDLE:
                    middle_extended = True

            if (
                number_extended_fingers == 2
                and index_extended == True
                and middle_extended == True
                and index_middle_tip_distance > 30
            ):
                if peace_extended == False:
                    peace_extended_time = time.time()
                peace_extended = True
                if peace_extended == True and time.time() - peace_extended_time > 1:
                    print("Peace Sign!!!")
                else:
                    print("Pending peace sign :0")
                # print(index_middle_tip_distance)
            else:
                print("No Peace sign yet :(((")
                peace_extended = False
                # print('Number of Extended Fingers: %s; Index: %s, Middle: %s' % (number_extended_fingers,index_extended,middle_extended))
            # print(len(extended_finger_list))


if __name__ == "__main__":
    main()