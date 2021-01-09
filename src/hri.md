# Human Robot Interaction

# Hand Gestures

Hand gestures are used to set the robot's different 'modes' for positioning and scanning the workpiece grooves. A Leap Motion Controller developed by the Ultraleap company are leveraged in order to detect such gestures.

The Leap Motion controller makes use of infrared cameras and software algorithms to tracked hands' skeletal models with detailed information on the pose of every bone and joint in the hand, as well as additional data such has left/right hand and whether a finger is extended. Using this data collected over time, we are able to recognize both static and dynamic gestures. 

## Static Gestures

By tracking the hand's pose at one instant, static gestures can be recognized.

### Hand open
```python
# All fingers extended.
if len(fingers_extended_list) == 5:
  hand_open_gesture = True
```

### Hand Closed

```python
# All fingers retracted.
if len(fingers_extended_list) == 0:
  hand_closed_gesture = True
```

### OK Sign

```python
# Thumb and index finger retracted and distance is smaller than 30mm. Middle, ring, pinky finger extended
if (
    middle in fingers_extended_list
    and pinky in fingers_extended_list
    and ring in fingers_extended_list
    and index_thumb_distance < 30 # Milimeteres
    ):
    ok_sign_gesture = True

```

### Peace Sign

```python
# Middle and index finger extended and distance is smaller than 30mm. Thumb, ring, pinky finger retracted.
if (
    thumb not in fingers_extended_list
    and ring not in fingers_extended_list
    and pinky not in fingers_extended_list
    and index_middle_finger_distance < 30 # Milimeteres
):
    peace_sign_gesture = True
```

## Dynamic Gestures

By tracking the hand's pose over time, dynamic gestures can be recognized as well.

### Hand wave

The hand's direction vector, i.e. the direction from palm to fingers, projected to the Leap Motion Controller's XY plane can be tracked over time and represented as a changing angle. When plotted, it looks like this. A hand wave can then be characterized as a high amplitude and high frequency signal derived from the hand direction's changing angle.

```python
time_series_hand_poses = collect_hand_tracking_data()
time_series_hand_angles = calculate_hand_angles(time_series_hand_poses)
plot(time_series_hand_angles)
signal_frequency, singal_amplitude = signal_analysis(time_series_hand_angles)
if (
    signal_freqency > detection_threshold_1
    and hand_wave_detection_threshold > detection_threshold_2
):
    hand_wave_detected = True

```

# Robot Tracking algorithm

```python
# Measure the pose (position & orientation) of the hand relative to the Leap Motion Controller
relative_hand_pose = get_hand_pose()

# Retrieve the pose of the UR3 arm and use it to transform the relative position of the hand into the absolute coordinate frame of the UR3 Arm
robot_pose = get_robot_pose()
absolute_hand_pose = get_absolute_hand_pose(robot_pose, relative_hand_pose)

# Define a basis/coordinate system based on the hand
# Z-Axis: Direction of fingers from palm
# Y-axis: Direction of palm normal
# X-axis: Direction of palm to pinky
hand_basis = calculate_coordinate_system(absolute_hand_pose)

# Derive a wanted basis of the robot mirroring the hand basis
wanted_robot_basis = calculate_wanted_basis(absolute_hand_pose)

# Convert the wanted basis into a roation vector
wanted_robot_rotation_vector = calculate_rotation_vector(wanted_robot_basis)

# Calculate the wanted position of the robot a set distance from the Y-component of the hand basis, i.e. the vector pointing out from the palm.
wanted_robot_position = calculate_wanted_robot_position(absolute_hand_pose, robot_hand_distance)

# Send the a move instruction encoding the rotation vector and new position to the UR robot
move_robot(wanted_robot_position, wanted_robot_position)
```




