import sys
sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")
import math
import time
import Leap
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt

def main():
  # Initialize leap motion controller object
  controller = Leap.Controller()

  # Breaks loop and stops script when wave is detected
  wave_detected = False
  while not wave_detected:
    # Initialize arrays for storing hand orientation data
    hand_direction_x = []
    hand_direction_y = []
    hand_direction_z = []
    extended_fingers = []

    # Captures hand orientation data in 1 second windows
    # 
    # Means you will need to be waving for an entire 1 second window duration for the 
    # wave to be detected
    start_time = time.time()
    while time.time() < start_time + 1:
      # Polls controller for frame containing hand/bones/joints data
      frame = controller.frame()
      if len(frame.hands):
        # Get the direction of hand in frame
        #### TO-DO: Deal with 2 hands in frame ####
        hand_direction = frame.hands[0].direction
        print(hand_direction)
        # Store hand orientations; note Z is inverted
        hand_direction_x.append(hand_direction[0])
        hand_direction_y.append(hand_direction[1])
        hand_direction_z.append(-1 * hand_direction[2])
        extended_fingers.append(len(frame.fingers.extended()))
            
    # Hard-coded angle calculation from captured hand orientation vectors
    #### TO-DO: Clean this up! :)
    angle_array = []
    for i in range(len(hand_direction_x)):
      angle = 0
      if hand_direction_x[i] > 0:
        angle = math.atan(hand_direction_x[i] / hand_direction_z[i])
        if angle < 0:
          angle = 3.14159265359 - (-1 * angle)
      else:
        angle = 1 * math.atan(hand_direction_x[i] / hand_direction_z[i])
        if angle > 0:
          angle = -1 * (3.14159265359 - (1 * angle))
      angle_array.append(angle)
    # Convert to np array
    angle_array = np.array(angle_array)

    # Smooth angles data with moving average filter with window size: *19*
    if len(angle_array) > 0: 
      angle_array = np.convolve(angle_array, np.ones(19), 'valid') / 19

    # Set divider between peak and trough detection values to mean max&min angles
    peak_base = ((max(angle_array) + min(angle_array)) / 2) if len(angle_array) > 0 else 0

    # Calculate peaks and troughs angle values 
    peaks, _ = find_peaks(angle_array, height=peak_base)
    troughs, _ = find_peaks(-angle_array, height=peak_base)

    # wave_gesture_amplitude is calulated by difference between average angles of peaks and troughs
    average_peaks = np.average(angle_array[peaks])
    average_troughs = np.average(angle_array[troughs])
    wave_gesture_amplitude = abs(average_peaks - average_troughs)

    # wave count is calucalted by average number of peaks and troughs
    wave_count = (len(peaks) + len(troughs)) / 2
    # print('Average Count Peaks & Troughs:   %s!' % (wave_count))

    average_extended_fingers = np.mean(extended_fingers)
    # Wave is detected if the wave gesture amplitude and wave count exceed a set threshold
    wave_detected = True if wave_gesture_amplitude > 0.55 and wave_count >= 3 and average_extended_fingers >= 4.4 else False
    if wave_detected:
      print("\nHand wave detected! :) \n")
      print(len(hand_direction_x))
      print(len(angle_array))
      plt.plot(angle_array, 'r-')
      plt.plot(peaks, angle_array[peaks], "x")
      plt.plot(troughs, angle_array[troughs], "o")
      plt.show()
    else:
      print("No hand wave detected :(")

            
if __name__ == "__main__":
  main()