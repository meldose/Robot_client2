#!/usr/bin/env python3
import socket
import time
import CommunicationLibrary
import random
import math
import numpy as np

SOCKET_RECV_TIMEOUT = 5
ROBOT_CONTROLLER_IP = "192.168.1.2"
PORT = 11004

# Requests
JOINT_STATE_TYPE = 1
TOOL_POSE_TYPE = 2

# variables for get_joint_state() + get_tool_pose()
init_joint_state = [0, 0, 0, 0, 0, 0]
base_quat = np.array([1, 0, 0, 0])


def get_joint_state(init_joint_state):
    if not hasattr(get_joint_state, 'joint_state'):
        get_joint_state.joint_state = init_joint_state # create function attribute for storing actual joint_state
        print(f"Initialized: {get_joint_state.joint_state}")
        get_joint_state.counter = 0 # create counter for switching

    # increments of joints
    divider = 100  # affect size of increment - inverse proportion
    inc_j1 = (random.random() * math.pi / divider)
    inc_j2 = (random.random() * math.pi / divider)
    inc_j3 = (random.random() * math.pi / divider)
    inc_j4 = (random.random() * math.pi / divider)
    inc_j5 = (random.random() * math.pi / divider)
    inc_j6 = (random.random() * math.pi / divider)

    # increment +/- number
    pos_neg = [1, 1, 1, 1, 1, 1] # affect the sign of increment
    # set different combinations for changing robot motions
    if get_joint_state.counter < 30:
        pos_neg = [-1, 1, 1, 1, 1, -1]
    elif get_joint_state.counter < 60:
        pos_neg = [1, 0, -1, -1, -1, 1]
    elif get_joint_state.counter < 90:
        pos_neg = [1, -1, 1, 1, -1, 1]
    elif get_joint_state.counter < 120:
        pos_neg = [1, 0, -1, -1, 1, -1]
        get_joint_state.counter = 0

    # incrementing joint_state
    get_joint_state.joint_state = [
        get_joint_state.joint_state[0] + inc_j1 * pos_neg[0],
        get_joint_state.joint_state[1] + inc_j2 * pos_neg[1],
        get_joint_state.joint_state[2] + inc_j3 * pos_neg[2],
        get_joint_state.joint_state[3] + inc_j4 * pos_neg[3],
        get_joint_state.joint_state[4] + inc_j5 * pos_neg[4],
        get_joint_state.joint_state[5] + inc_j6 * pos_neg[5]]

    # set joint limits
    upper_limit = [3.14, 0.6, 1.13, 3.14, 0.6, 3.14]
    lower_limit = [0, -0.6, -3, 0, -0.6, -3.14]
    count = 0
    # joint limits check
    for joint_value in get_joint_state.joint_state:
        if joint_value > upper_limit[count]:
            get_joint_state.joint_state[count] = upper_limit[count]
        elif joint_value < lower_limit[count]:
            get_joint_state.joint_state[count] = lower_limit[count]
        count += 1

    # increment counter
    get_joint_state.counter += 1
    return get_joint_state.joint_state


def get_tool_pose(init_quaternion):
    if not hasattr(get_tool_pose, 'actual_quaternion'):
        get_tool_pose.actual_quaternion = init_quaternion # create function attribute to store actual quaternion value
        print(f"Initialized: {get_tool_pose.actual_quaternion}")
        # create counters for functions
        get_tool_pose.counter = 0  # changing rotation
        get_tool_pose.counter_circle = 0  # circular motion

    # define circular motion of tool_pose - around z-axis
    num_frames = 100
    radius = 1000
    alfa = 2 * np.pi * get_tool_pose.counter_circle / num_frames
    x = radius * np.cos(alfa)
    y = radius * np.sin(alfa)
    z = 500

    # Define the rotation of quaternion
    theta = np.pi / 50  # radians
    rot = np.sin(theta / 2)
    rotation_quat = np.array([np.cos(theta / 2), 0, 0, 0]) # no rotation

    if get_tool_pose.counter < num_frames / 4:
        rotation_quat = np.array([np.cos(theta / 2), 0, 0, rot])  # z-axis
    elif get_tool_pose.counter < 2 * num_frames / 4:
        rotation_quat = np.array([np.cos(theta / 2), 0, rot, 0])  # y-axis
    elif get_tool_pose.counter < 3 * num_frames / 4:
        rotation_quat = np.array([np.cos(theta / 2), rot, 0, 0])  # x-axis
    elif get_tool_pose.counter < num_frames:
        rotation_quat = np.array([np.cos(theta / 2), 0, 0, 0])  # no rotation
        get_tool_pose.counter = 0  # reset counter


    rotation_quat = normalize_quaternion(rotation_quat) # Normalize the rotation quaternion (just in case)

    result_quat = quaternion_multiply(get_tool_pose.actual_quaternion, rotation_quat) # Apply the rotation to the actual quaternion

    get_tool_pose.actual_quaternion = normalize_quaternion(result_quat) # Normalize the resulting quaternion (just in case)

    # increment counters
    get_tool_pose.counter += 1
    get_tool_pose.counter_circle += 1

    return [x, y, z, result_quat[0], result_quat[1], result_quat[2], result_quat[3]]


# Function to multiply two quaternions
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])


# Function to normalize a quaternion
def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    return q / norm


def test_loop_communication():
    server = CommunicationLibrary.RobotStateCommunication() # create server object
    server.create_server(ROBOT_CONTROLLER_IP, PORT)
    server.wait_for_client()
    while True:
        try:
            server.send_joint_state()
            server.send_tool_pose()
        except socket.error:
            print('Communication lost. Trying to reconnect...')
            break

        time.sleep(0.1)


if __name__ == "__main__":
    test_loop_communication()
