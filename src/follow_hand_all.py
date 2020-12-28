import sys
sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")
import urx
import time
import Leap
import math
import numpy as np
import math3d as m3d
from scipy.spatial.transform import Rotation as R

# Converts URx's rotation vector into a rotation matrix
#
# I did not derive this nor do I fully understand the maths behind this :0
# I took it from: https://dof.robotiq.com/discussion/1648/around-which-axes-are-the-rotation-vector-angles-defined
def convert_tool_pose_to_transformation_matrix(tool_pose):
    position_vector = np.array(tool_pose[:3]).reshape((3,1))
    rotation_vector = np.array(tool_pose[3:])
    
    rotation_matrix = R.from_rotvec(rotation_vector).as_dcm()

    transformation_matrix = np.append(rotation_matrix, position_vector, axis = 1)
    transformation_matrix = np.append(transformation_matrix ,np.array([[0,0,0,1]]), axis=0)
    return transformation_matrix

def calculate_rotation_matrix(rotation_vector):
    return R.from_rotvec(np.array(rotation_vector)).as_dcm()

def calculate_coordinate_system_from_hand():
    pass

def convert_coordinate_spaces_to_rotation_vector(A_prime):
    A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    rotation_matrix = np.dot(A, np.linalg.inv(A_prime))
    r = R.from_matrix(rotation_matrix)
    return r.as_rotvec

# Calculates hand position in absolute coordinates
def calculate_hand_position(transformation_matrix, relative_palm_postion):
    # Formats raw hand coordinates
    hand_coordinates_raw = relative_palm_postion
    # m to mm converstion
    hand_coordinates = (np.array(hand_coordinates_raw) * [-1, -1, 1]) / 1000
    
    # Converts to auguemented position vector 
    hand_coordinates = np.append(hand_coordinates, [1])

    # Gets abolsolute matrix by transformation matrix multiplication
    absolute_position = transformation_matrix.dot(hand_coordinates)
    return np.round(absolute_position[:3], 3)


def calculate_required_robot_position(absolute_hand_position, y_offset=0):
    required_robot_position = absolute_hand_position + [0, 0.19, 0]
    # required_robot_position = absolute_hand_position + y_offset
    return required_robot_position

def get_tool_pose(robot):
    cartesian_info = robot.secmon.get_all_data()['CartesianInfo']
    tool_pose = [cartesian_info['X'],cartesian_info['Y'],cartesian_info['Z'],cartesian_info['Rx'],cartesian_info['Ry'],cartesian_info['Rz']]
    # tool_pose = [50, -600, -135, 0, 3.14, 0]
    return tool_pose

def read_hand_position(frame):
    return list(frame.hands[0].palm_position.to_tuple())


def main():
    # Leap motion 
    controller = Leap.Controller()
    controller.config.save()

    # UR3 robot config
    robot = urx.Robot("192.168.0.2")
    mytcp = m3d.Transform()  # create a matrix for our tool tcp
    mytcp.pos.x = -0.0002
    mytcp.pos.y = -0.144
    mytcp.pos.z = 0.05
    time.sleep(1)
    robot.set_tcp(mytcp)
    time.sleep(1)

    while 1:
        try:
            tool_pose = get_tool_pose(robot)
            T = convert_tool_pose_to_transformation_matrix(tool_pose)

            frame = controller.frame()
            if len(frame.hands):
                extended_finger_list = frame.fingers.extended()
                number_extended_fingers = len(extended_finger_list)
                if (
                    number_extended_fingers != 5
                ):
                    pass
                else:
                    relative_palm_postion = read_hand_position(frame)
                    absolute_hand_position = calculate_hand_position(T, relative_palm_postion)

                    required_robot_position = calculate_required_robot_position(absolute_hand_position)

                    final_pose = list(required_robot_position)
                    final_pose.extend(tool_pose[3:])

                    pose_difference = np.linalg.norm(np.array(tool_pose[:3]) - np.array(required_robot_position))

                    # Only moves robot if the move is greater than 0.5cm; reduces jitter this way
                    if pose_difference > 0.005:
                        print('\ncurrent_pose: %s' % (tool_pose))
                        print('\nabsolute_hand_position: %s' % (absolute_hand_position))
                        print('required_pose: %s' % (final_pose))
                        print('pose_difference: %s' % (pose_difference))
                        # Only moves robot if move is smaller than 8cm, minimizes robot moving in strange directions
                        if pose_difference < 1:
                            robot.movep(list(final_pose), acc=0.1, vel=0.1, wait=False)

        except:
            robot.close()
            sys.exit(0)


def get_hand_basis(frame):
    basis_raw = frame.hands[0].basis
    # X - positive to pinky
    # Y - positive slap
    # Z - positive thrust
    basis = [list(basis_raw.x_basis.to_tuple()), list(basis_raw.y_basis.to_tuple()),  list(basis_raw.z_basis.to_tuple())]
    return basis

def test_transform():
    # Leap motion 
    controller = Leap.Controller()
    controller.config.save()
    ### Orientation
    # Get hand basis matrix
    robot = urx.Robot("192.168.0.2")
    mytcp = m3d.Transform()  # create a matrix for our tool tcp
    mytcp.pos.x = -0.0002
    mytcp.pos.y = -0.144
    mytcp.pos.z = 0.05
    time.sleep(1)
    robot.set_tcp(mytcp)
    time.sleep(1)

    while 1:
        try:
            tool_pose = get_tool_pose(robot)
            T = convert_tool_pose_to_transformation_matrix(tool_pose)
            # print(tool_pose[3:])
            frame = controller.frame()
            basis = get_hand_basis(frame)
            if frame.hands[0].is_left:
                print('LEFT')
                for i in range(3):
                    basis[0][i] = basis[0][i] * -1
            # print(basis)
            rotation_matrix = calculate_rotation_matrix(tool_pose[3:])
            # basis_corrected = np.array(basis)
            hand_basis_absolute = rotation_matrix.dot(np.array(basis).T)
            # hand_basis_absolute = rotation_matrix.dot(np.array(basis))

            # Basis is in a left-handed coordinate system,lets convert to right
            hand_basis_absolute[0] = hand_basis_absolute[0] * np.array([-1, -1, 1])

            orientation_matrix = np.array([[1,0,0],[0,1,0],[0,0,-1]])
            final_basis = np.dot(hand_basis_absolute,orientation_matrix)
            # print(final_basis)
            final_basis_z_locked = []
            for i in range(len(final_basis)):
                row = final_basis[i]
                if i == 2:
                    row[0] = 0
                    row[1] = 0
                    row[2] = 1
                else:
                    row[2] = 0
                final_basis_z_locked.append(row / np.linalg.norm(row))
            final_basis_z_locked = np.array(final_basis_z_locked)

            robot_orientation_matrix = np.array([[1,0,0],[0,1,0],[0,0,-1]])
            wanted_basis = np.dot(final_basis,robot_orientation_matrix)


            coordinate_system = np.array([[-1,0,0],[0,1,0],[0,0,1]])
            inv_coordinate_system = np.linalg.inv(coordinate_system)

            wanted_rotation_matrix = np.dot(inv_coordinate_system,wanted_basis)
            wanted_rotation_vector = R.from_dcm(wanted_rotation_matrix).as_rotvec()

            relative_palm_postion = read_hand_position(frame)
            absolute_hand_position = calculate_hand_position(T, relative_palm_postion)
            # print(final_basis_z_locked)
            required_robot_position = (final_basis_z_locked[1] * 0.19) + np.array(absolute_hand_position)

            final_pose = list(np.append(required_robot_position, wanted_rotation_vector))
            print(final_pose)
            print('--------------------------------------------------------\n')
            robot.movep(final_pose, acc=0.1, vel=0.1, wait=False)
            # time.sleep(0.2)
            
        except:
            robot.close()
            sys.exit(0)

    # Calculate hand basis in absolute frame via rotation matrix
    # Lock hand basis to Z-axis
    # Flip x and y vectors of hand basis
    # Solve rotation matrix from coordiante frame to hand basis
    # convert rotation matrix to vector
    # basis_x[2] = 0
    # basis_y[2] = 0
    # basis_z = [0, 0, 1]

    ### Position
    # Get palm normal in absolute frame from ^^ steps
    # Get palm coordinates in abosulate frame
    # Get coordinates of set distance from palm along palm normal

    # Send pos and rotation matrix to UR robot 
    
    # Transform vectors to absolute
    # right hand via c
    # pass

def test_math():
    final_basis = np.array([[1,0,0], [0,1,0], [0,0,-1]])
    rotation_matrix = R.from_rotvec(np.array([0,3.14,0])).as_dcm()
    print( np.dot(  np.linalg.inv(rotation_matrix), final_basis))
    pass

if __name__ == "__main__":
    # main()
    test_transform()
    # test_math()
