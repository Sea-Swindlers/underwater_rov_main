#!/usr/bin/python3
import numpy as np
np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)


# boat_state = [x_rot, y_rot, z_rot]
boat_state = [np.array([0, 0, 0, 0, 0, 0])]

# motor_placements = [[position_x, position_y, position_z, axis_x, axis_y, axis_z], ...]
# motor positions are relative to the submarine center of mass
# motor_placements = [
#                     np.array([0, 0, 0, 1, 0, 0]),
#                     np.array([0, 0, 0, 0, 1, 0]),
#                     np.array([0, 0, 0, 0, 0, 1]),
#                     np.array([1, 0, 0, 1, 0, 0]),
#                     np.array([1, 0, 0, 0, 1, 0]),
#                     np.array([1, 0, 0, 0, 0, 1])
#                     ]

# # current motor placements
# motor_placements = [
#                     np.array([1, 0, 0, 0, 1, 0]),
#                     np.array([-1, 0, 0, 0, 1, 0]),
#                     np.array([1, 0, 0, 0, 0, 1]),
#                     np.array([-1, 0, 0, 0, 0, 1])
#                     ]

# more motors than necessary
motor_placements = [
                    np.array([1, 1, 0, -1, 1, 0]),
                    np.array([0, 1, 0, 0, 1, 0]),
                    np.array([-1, 1, 0, 1, 1, 0]),
                    np.array([0, 0, 0, 1, 0, 0]),
                    np.array([1, 0, 0, 0, 0, 1]),
                    np.array([1, 0, 0, 0, 0, 1]),
                    np.array([-1, 0, 0, 0, 0, 1])
                    ]          

def motor_to_wrench(motor: np.array) -> np.array:
    # the force component of the wrench is equal to the motor axis
    force = motor[3:6]
    force = force / np.linalg.norm(force)  # normalize the force axis vector
    # the torque is a little trickier. The cross product can be used for this
    moment = np.cross(motor[0:3], motor[3:6])
    return np.concatenate((force, moment))

# hope the motors approximate a basis for the space of twists. Construct the requested thrust from the set of motors
# TODO: make this a memoized function maybe? 
motor_basis = np.array([motor_to_wrench(i) for i in motor_placements]).T



def wrench_to_motor_values(requested_wrench, gains = np.ones([6])):
    requested_wrench_in_motor_basis = np.linalg.pinv(motor_basis).dot(requested_wrench)  # use pseudoinverse, don't assume that the motors form a basis
    return requested_wrench_in_motor_basis


def motor_values_to_wrench(motor_powers: np.array) -> np.array:
    resulting_wrench = motor_basis.dot(motor_powers)
    return resulting_wrench


def print_error(requested_twist):
    print("requested twist:         ", requested_twist)
    outputs = wrench_to_motor_values(requested_twist)
    print("calculated motor forces: ", outputs)
    resultant_twist = motor_values_to_wrench(outputs)
    print("resultant twist:         ", resultant_twist)

if __name__ == "__main__":
    print_error(np.array([1, 0, 0, 0, 0, 0]))
    print()
    print_error(np.array([0, 1, 0, 0, 0, 0]))
    print()
    print_error(np.array([0, 0, 1, 0, 0, 0]))
    print()
    print_error(np.array([0, 0, 0, 1, 0, 0]))
    print()
    print_error(np.array([0, 0, 0, 0, 1, 0]))
    print()
    print_error(np.array([0, 0, 0, 0, 0, 1]))