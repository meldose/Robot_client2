#!/usr/bin/env python3
import socket
import sys
from copy import deepcopy
import struct
import numpy as np
from RobotStateServer import get_joint_state, get_tool_pose, init_joint_state, base_quat

BRAND_IDENTIFICATION = "ABB_IRB/1.8.0XXXXXXXXXXX"
BRAND_IDENTIFICATION_SERVER = "ABB_IRB/1.8.0XXXXXXXXXXX"



class MessageType:
    PHO_TRAJECTORY_CNT = 0
    PHO_TRAJECTORY_FINE = 1
    PHO_GRIPPER = 2
    PHO_ERROR = 3
    PHO_INFO = 4
    PHO_OBJECT_POSE = 5


class ActionRequest:
    # Bin picking action requests
    PHO_BINPICKING_INITIALIZATION = 4
    PHO_BINPICKING_SCAN = 1
    PHO_BINPICKING_TRIGGER_SCAN = 28
    PHO_BINPICKING_LOCALIZE_ON_THE_LAST_SCAN = 29
    PHO_BINPICKING_TRAJECTORY = 2
    PHO_BINPICKING_PICK_FAILED = 7
    PHO_BINPICKING_OBJECT_POSE = 8
    PHO_BINPICKING_CHANGE_SCENE_STATE = 15
    PHO_BINPICKING_GET_VISION_SYSTEM_STATUS = 21
    # Locator action requests
    PHO_LOCATOR_SCAN = 19
    PHO_LOCATOR_TRIGGER_SCAN = 30
    PHO_LOCATOR_LOCALIZE_ON_THE_LAST_SCAN = 31
    PHO_LOCATOR_GET_OBJECTS = 20
    PHO_LOCATOR_GET_VISION_SYSTEM_STATUS = 22
    # Calibration action requests
    PHO_CALIBRATION_ADD_POINT = 5
    PHO_CALIBRATION_START_AUTOMATIC = 25
    PHO_CALIBRATION_SAVE_AUTOMATIC = 27
    PHO_CALIBRATION_STOP_AUTOMATIC = 26
    # Solution action requests
    PHO_SOLUTION_CHANGE = 9
    PHO_SOLUTION_START = 10
    PHO_SOLUTION_STOP = 11
    PHO_SOLUTION_GET_RUNNING = 12
    PHO_SOLUTION_GET_AVAILABLE = 13


request_name = {
    # Bin picking action requests
    ActionRequest.PHO_BINPICKING_INITIALIZATION: "INITIALIZATION [BINPICKING]",
    ActionRequest.PHO_BINPICKING_SCAN: "SCAN [BINPICKING]",
    ActionRequest.PHO_BINPICKING_TRIGGER_SCAN: "TRIGGER SCAN [BINPICKING]",
    ActionRequest.PHO_BINPICKING_LOCALIZE_ON_THE_LAST_SCAN: "LOCALIZE ON THE LAST SCAN [BINPICKING]",
    ActionRequest.PHO_BINPICKING_TRAJECTORY: "TRAJECTORY [BINPICKING]",
    ActionRequest.PHO_BINPICKING_PICK_FAILED: "PICK-FAILED [BINPICKING]",
    ActionRequest.PHO_BINPICKING_OBJECT_POSE: "OBJECT POSE [BINPICKING]",
    ActionRequest.PHO_BINPICKING_CHANGE_SCENE_STATE: "CHANGE SCENE STATE [BINPICKING]",
    ActionRequest.PHO_BINPICKING_GET_VISION_SYSTEM_STATUS: "GET VISION SYSTEM STATUS [BINPICKING]",
    # Locator action requests
    ActionRequest.PHO_LOCATOR_SCAN: "SCAN [LOCATOR]",
    ActionRequest.PHO_LOCATOR_TRIGGER_SCAN: "TRIGGER SCAN [LOCATOR]",
    ActionRequest.PHO_LOCATOR_LOCALIZE_ON_THE_LAST_SCAN: "LOCALIZE ON THE LAST SCAN [LOCATOR]",
    ActionRequest.PHO_LOCATOR_GET_OBJECTS: "GET OBJECTS [LOCATOR]",
    ActionRequest.PHO_LOCATOR_GET_VISION_SYSTEM_STATUS: "GET VISION SYSTEM STATUS [LOCATOR]",
    # Calibration action requests
    ActionRequest.PHO_CALIBRATION_ADD_POINT: "ADD CALIBRATION POINT",
    ActionRequest.PHO_CALIBRATION_START_AUTOMATIC: "START AUTOMATIC CALIBRATION",
    ActionRequest.PHO_CALIBRATION_SAVE_AUTOMATIC: "SAVE AUTOMATIC CALIBRATION RESULT",
    ActionRequest.PHO_CALIBRATION_STOP_AUTOMATIC: "STOP AUTOMATIC CALIBRATION",
    # Solution action requests
    ActionRequest.PHO_SOLUTION_CHANGE: "CHANGE SOLUTION",
    ActionRequest.PHO_SOLUTION_START: "START SOLUTION",
    ActionRequest.PHO_SOLUTION_STOP: "STOP SOLUTION",
    ActionRequest.PHO_SOLUTION_GET_RUNNING: "GET RUNNING SOLUTION",
    ActionRequest.PHO_SOLUTION_GET_AVAILABLE: "GET AVAILABLE SOLUTION"}

# STATE SERVER Requests
JOINT_STATE_TYPE = 1
TOOL_POSE_TYPE = 2

# sizes
HEADER_SIZE = 12
SUBHEADER_SIZE = 12
PACKET_SIZE = 4
NUMBER_OF_JOINTS = 6
CARTES_POSE_LEN = 7

# Photoneo header
PHO_HEADER = struct.pack("III", 80, 72, 79)  # P, H, O


class ResponseHeader:
    def __init__(self, request_id, sub_headers):
        self.request_id = request_id
        self.sub_headers = sub_headers


class ResponseData:  # class used for storing data

    def __init__(self):
        self.segment_id = 0

    print_message = 1  # set 1 for printing messages
    error = 0
    gripper_command = None  # stores gripper commands
    trajectory_data = []  # stores trajectory waypoints in 4 segments
    gripping_info = None
    dimensions = []
    status_data = None
    calib_data = None
    running_solution = None
    available_solution = []
    object_pose = []
    camera_pose = []
    zheight_angle = [] # ako to pomenovat ??

    def init_response_data(self):
        self.error = 0
        #self.gripper_command = []  # stores gripper commands
        #self.trajectory_data = []  # stores trajectory waypoints in 4 segments
        self.gripping_info = []
        self.dimensions = []
        self.status_data = None
        self.calib_data = None
        self.running_solution = None
        self.object_pose = []

    def init_trajectory_data(self):
        # empty the variable for storing trajectory
        self.trajectory_data = []
        self.trajectory_data.append(np.empty((0, NUMBER_OF_JOINTS), dtype=float))
        self.segment_id = 0
        self.gripper_command = []

    def add_waypoint(self, slice_index, row):
        self.trajectory_data[slice_index] = np.vstack([self.trajectory_data[slice_index], row])

    def add_segment(self):
        self.trajectory_data.append(np.empty((0, NUMBER_OF_JOINTS), dtype=float))

    def data_store(self, message_type, request_id, message): #store received messages into variables - specific for each request
        if message_type == MessageType.PHO_TRAJECTORY_CNT or message_type == MessageType.PHO_TRAJECTORY_FINE:
            if self.print_message == 1: print("trajectory: " + str(self.trajectory_data))
        elif message_type == MessageType.PHO_GRIPPER:
            if self.print_message == 1: print("gripper commands: " + str(self.gripper_command))
        elif message_type == MessageType.PHO_ERROR:
            self.error = message
            if self.print_message == 1: print("error message: " + str(self.error))
        elif message_type == MessageType.PHO_INFO:
            # TRAJECTORY - BPS
            if request_id == ActionRequest.PHO_BINPICKING_TRAJECTORY:
                self.gripping_info.append(message)
                if self.print_message == 1: print("gripping info: " + str(self.gripping_info))
            # OBJECT POSE - BPS
            elif request_id == ActionRequest.PHO_BINPICKING_OBJECT_POSE:
                self.dimensions = message
                if self.print_message == 1: print("dimensions: " + str(self.dimensions))
            # GET VISION SYSTEM STATUS
            elif request_id == ActionRequest.PHO_BINPICKING_GET_VISION_SYSTEM_STATUS:
                self.status_data = message
                if self.print_message == 1: print("status data: " + str(self.status_data))
            # GET OBJECTS - LS
            elif request_id == ActionRequest.PHO_LOCATOR_GET_OBJECTS:
                self.dimensions.append(message)
                if self.print_message == 1: print("dimensions: " + str(self.dimensions))
            elif request_id == ActionRequest.PHO_LOCATOR_GET_VISION_SYSTEM_STATUS:
                self.status_data = message
                if self.print_message == 1: print("status data: " + str(self.status_data))
            elif request_id == ActionRequest.PHO_CALIBRATION_SAVE_AUTOMATIC:
                self.calib_data = message
                if self.print_message == 1: print("calibration data: " + str(self.calib_data))
            elif request_id == ActionRequest.PHO_SOLUTION_GET_RUNNING:
                self.running_solution = message
                if self.print_message == 1: print("running solution: " + str(self.running_solution))

        elif message_type == MessageType.PHO_OBJECT_POSE:
            self.object_pose.append(message)
            if self.print_message == 1: print("object pose: " + str(self.object_pose))
        else:
            print('\033[31mUnexpected operation type\033[0m')
            sys.exit()



class RobotRequestResponseCommunication:
    response_data = ResponseData()  # create object for storing data

    def __init__(self):
        self.active_request = 0  # variable to check, if old request has finished and new one can be called
        self.client = None
        self.message = None
        self.print_messages = True  # True -> prints messages , False -> doesnt print messages

    def connect_to_server(self, CONTROLLER_IP, PORT):
        self.client = socket.socket()
        self.client.connect((str(CONTROLLER_IP), PORT))
        msg = bytearray(BRAND_IDENTIFICATION.encode('utf-8'))
        self.client.send(msg)

    def close_connection(self):
        self.client.close()

    # -------------------------------------------------------------------
    #                      BIN PICKING REQUESTS
    # -------------------------------------------------------------------
    def pho_request_binpicking_init(self, vs_id, start, end):
        payload = struct.pack("i", vs_id)  # payload - vision system ID
        payload = payload + floatArray2bytes(start)  # payload - robot start pose
        payload = payload + floatArray2bytes(end)  # payload - robot end pose
        self.pho_send_request(ActionRequest.PHO_BINPICKING_INITIALIZATION, payload)
        self.pho_receive_response(ActionRequest.PHO_BINPICKING_INITIALIZATION)

    def pho_request_binpicking_scan(self, vs_id, tool_pose=None):
        if tool_pose is None:
            payload = struct.pack("i", vs_id)  # payload - vision system ID
            self.pho_send_request(ActionRequest.PHO_BINPICKING_SCAN, payload)

        else:
            payload = struct.pack("i", vs_id)  # payload - vision system id
            payload = payload + floatArray2bytes(tool_pose)  # payload - robot pose
            self.pho_send_request(ActionRequest.PHO_BINPICKING_SCAN, payload)

    def pho_request_binpicking_trigger_scan(self, vs_id, tool_pose=None):
        if tool_pose is None:
            payload = struct.pack("i", vs_id)  # payload - vision system ID
            self.pho_send_request(ActionRequest.PHO_BINPICKING_TRIGGER_SCAN, payload)
        else:
            payload = struct.pack("i", vs_id)  # payload - vision system id
            payload = payload + floatArray2bytes(tool_pose)  # payload - robot pose
            self.pho_send_request(ActionRequest.PHO_BINPICKING_TRIGGER_SCAN, payload)

    def pho_request_binpicking_localize_on_the_last_scan(self, vs_id, tool_pose=None):
        if tool_pose is None:
            payload = struct.pack("i", vs_id)  # payload - vision system ID
            self.pho_send_request(ActionRequest.PHO_BINPICKING_LOCALIZE_ON_THE_LAST_SCAN, payload)
        else:
            payload = struct.pack("i", vs_id)  # payload - vision system id
            payload = payload + floatArray2bytes(tool_pose)  # payload - robot pose
            self.pho_send_request(ActionRequest.PHO_BINPICKING_LOCALIZE_ON_THE_LAST_SCAN, payload)

    def pho_binpicking_wait_for_scan(self):
        self.pho_receive_response(ActionRequest.PHO_BINPICKING_SCAN)
        self.active_request = 0  # request finished - response from request received

    def pho_request_binpicking_trajectory(self, vs_id):
        payload = struct.pack("i", vs_id)  # payload - vision system ID
        self.pho_send_request(ActionRequest.PHO_BINPICKING_TRAJECTORY, payload)
        self.pho_receive_response(ActionRequest.PHO_BINPICKING_TRAJECTORY)

    def pho_request_binpicking_pick_failed(self, vs_id):
        payload = struct.pack("i", vs_id)  # payload - vision system ID
        self.pho_send_request(ActionRequest.PHO_BINPICKING_PICK_FAILED, payload)
        self.pho_receive_response(ActionRequest.PHO_BINPICKING_PICK_FAILED)

    def pho_request_binpicking_object_pose(self, vs_id):
        payload = struct.pack("i", vs_id)  # payload - vision system ID
        self.pho_send_request(ActionRequest.PHO_BINPICKING_OBJECT_POSE, payload)
        self.pho_receive_response(ActionRequest.PHO_BINPICKING_OBJECT_POSE)

    def pho_request_binpicking_change_scene_status(self, scene_status_id):
        payload = struct.pack("i", scene_status_id)  # payload - status scene ID
        self.pho_send_request(ActionRequest.PHO_BINPICKING_GET_VISION_SYSTEM_STATUS, payload)
        self.pho_receive_response(ActionRequest.PHO_BINPICKING_GET_VISION_SYSTEM_STATUS)

    def pho_request_binpicking_get_vision_system_status(self, vs_id):
        payload = struct.pack("i", vs_id)  # payload - vision system id
        self.pho_send_request(ActionRequest.PHO_BINPICKING_GET_VISION_SYSTEM_STATUS, payload)
        self.pho_receive_response(ActionRequest.PHO_BINPICKING_GET_VISION_SYSTEM_STATUS)

    # -------------------------------------------------------------------
    #                      LOCATOR REQUESTS
    # -------------------------------------------------------------------

    # parameter tool_pose used only in Hand-eye
    def pho_request_locator_scan(self, vs_id, tool_pose=None):
        if tool_pose is None:
            payload = struct.pack("i", vs_id)  # payload - vision system id
            self.pho_send_request(ActionRequest.PHO_LOCATOR_SCAN, payload)
        else:
            if len(tool_pose) != 7:
                print('Wrong tool_pose size')
                sys.exit()
            payload = struct.pack("i", vs_id)  # payload - vision system id
            payload = payload + floatArray2bytes(tool_pose)  # payload - tool pose
            self.pho_send_request(ActionRequest.PHO_LOCATOR_SCAN, payload)

    def pho_locator_wait_for_scan(self):
        self.pho_receive_response(ActionRequest.PHO_LOCATOR_SCAN)
        self.active_request = 0  # request finished - response from request received

    def pho_request_locator_trigger_scan(self, vs_id, tool_pose=None):
        if tool_pose is None:
            payload = struct.pack("i", vs_id)  # payload - vision system ID
            self.pho_send_request(ActionRequest.PHO_LOCATOR_TRIGGER_SCAN, payload)
        else:
            payload = struct.pack("i", vs_id)  # payload - vision system id
            payload = payload + floatArray2bytes(tool_pose)  # payload - robot pose
            self.pho_send_request(ActionRequest.PHO_LOCATOR_TRIGGER_SCAN, payload)

    def pho_request_locator_localize_on_the_last_scan(self, vs_id, tool_pose=None):
        if tool_pose is None:
            payload = struct.pack("i", vs_id)  # payload - vision system ID
            self.pho_send_request(ActionRequest.PHO_LOCATOR_LOCALIZE_ON_THE_LAST_SCAN, payload)
        else:
            payload = struct.pack("i", vs_id)  # payload - vision system id
            payload = payload + floatArray2bytes(tool_pose)  # payload - robot pose
            self.pho_send_request(ActionRequest.PHO_LOCATOR_LOCALIZE_ON_THE_LAST_SCAN, payload)

    def pho_request_locator_get_objects(self, vs_id, number_of_objects):
        payload = struct.pack("ii", vs_id, number_of_objects)  # payload - vision system id, number of objects
        self.pho_send_request(ActionRequest.PHO_LOCATOR_GET_OBJECTS, payload)
        self.pho_receive_response(ActionRequest.PHO_LOCATOR_GET_OBJECTS)

    def pho_request_locator_get_vision_system_status(self, vs_id):
        payload = struct.pack("i", vs_id)  # payload - vision system id
        self.pho_send_request(ActionRequest.PHO_LOCATOR_GET_VISION_SYSTEM_STATUS, payload)
        self.pho_receive_response(ActionRequest.PHO_LOCATOR_GET_VISION_SYSTEM_STATUS)

    # -------------------------------------------------------------------
    #                      CALIBRATION REQUESTS
    # -------------------------------------------------------------------
    def pho_request_calibration_add_point(self, tool_pose=None):
        if tool_pose is None:
            self.pho_send_request(ActionRequest.PHO_CALIBRATION_ADD_POINT)
            self.pho_receive_response(ActionRequest.PHO_CALIBRATION_ADD_POINT)
        else:
            payload = floatArray2bytes(tool_pose)  # payload - robot pose
            self.pho_send_request(ActionRequest.PHO_CALIBRATION_ADD_POINT, payload)
            self.pho_receive_response(ActionRequest.PHO_CALIBRATION_ADD_POINT)

    def pho_request_calibration_start(self, sol_id, vs_id):
        payload = struct.pack("ii", sol_id, vs_id)  # payload - solution id, vision system id
        self.pho_send_request(ActionRequest.PHO_CALIBRATION_START_AUTOMATIC, payload)
        self.pho_receive_response(ActionRequest.PHO_CALIBRATION_START_AUTOMATIC)

    def pho_request_calibration_save(self):
        self.pho_send_request(ActionRequest.PHO_CALIBRATION_SAVE_AUTOMATIC)
        self.pho_receive_response(ActionRequest.PHO_CALIBRATION_SAVE_AUTOMATIC)

    def pho_request_calibration_stop(self):
        self.pho_send_request(ActionRequest.PHO_CALIBRATION_STOP_AUTOMATIC)
        self.pho_receive_response(ActionRequest.PHO_CALIBRATION_STOP_AUTOMATIC)

    # -------------------------------------------------------------------
    #                      SOLUTION REQUESTS
    # -------------------------------------------------------------------
    def pho_request_solution_change(self, sol_id):
        payload = struct.pack("i", sol_id)  # payload - vision system id
        self.pho_send_request(ActionRequest.PHO_SOLUTION_CHANGE, payload)
        self.pho_receive_response(ActionRequest.PHO_SOLUTION_CHANGE)

    def pho_request_solution_start(self, sol_id):
        payload = struct.pack("i", sol_id)  # payload - vision system id
        self.pho_send_request(ActionRequest.PHO_SOLUTION_START, payload)
        self.pho_receive_response(ActionRequest.PHO_SOLUTION_START)

    def pho_request_solution_stop(self):
        self.pho_send_request(ActionRequest.PHO_SOLUTION_STOP)
        self.pho_receive_response(ActionRequest.PHO_SOLUTION_STOP)

    def pho_request_solution_get_running(self):
        self.pho_send_request(ActionRequest.PHO_SOLUTION_GET_RUNNING)
        self.pho_receive_response(ActionRequest.PHO_SOLUTION_GET_RUNNING)

    def pho_request_solution_get_available(self):
        self.pho_send_request(ActionRequest.PHO_SOLUTION_GET_AVAILABLE)
        self.pho_receive_response(ActionRequest.PHO_SOLUTION_GET_AVAILABLE)

    # -------------------------------------------------------------------
    #                     REQUEST RELATED FUNCTIONS
    # -------------------------------------------------------------------

    def pho_send_request(self, request_id, payload=None):
        print("Sending request \033[35m" + request_name[request_id] + "\033[0m")
        if self.active_request != 0:
            print(
                "\033[31mCannot send request " + request_name[request_id] + " because previous request " + request_name[
                    self.active_request] + " is not finished \033[0m")
            sys.exit()

        self.active_request = request_id
        msg = PHO_HEADER  # header - PHO
        if payload is not None:
            msg = msg + struct.pack("ii", int(len(payload) / PACKET_SIZE),
                                    request_id)  # header - payload size, request ID
            msg = msg + bytearray(payload)  # payload
        else:
            msg = msg + struct.pack("ii", 0, request_id)  # header - payload size, request ID
        self.client.send(bytearray(msg))

    def pho_receive_response(self, required_id):
        # receive header
        received_header = self.client.recv(HEADER_SIZE)
        request_id = int.from_bytes(received_header[0:3], "little")
        number_of_messages = int.from_bytes(received_header[4:7], "little")

        # Accept BINPICKING TRIGGER_SCAN as REQUEST_SCAN
        if request_id == 28 or request_id == 29: request_id = 1
        # Accept LOCATOR TRIGGER_SCAN as REQUEST_SCAN
        if request_id == 30 or request_id == 31: request_id = 19

        #check received header size
        if len(received_header) != HEADER_SIZE:
            print('\033[31mWrong header size\033[0m')
            sys.exit()

        # check request ID
        header = ResponseHeader(request_id, number_of_messages)
        if header.request_id != required_id:
            print('\033[31mWrong request id received\033[0m')
            sys.exit()

        if request_id == ActionRequest.PHO_BINPICKING_TRAJECTORY: self.response_data.init_trajectory_data()  # empty variable for receiving new trajectory

        # clear response_data variables
        self.response_data.init_response_data()


        for message_count in range(header.sub_headers):
            received_subheader = self.client.recv(SUBHEADER_SIZE)
            message_type = int.from_bytes(received_subheader[0:3], "little")
            operation_number = int.from_bytes(received_subheader[4:7], "little")
            payload_size = int.from_bytes(received_subheader[8:11], "little")
            # check received subheader size
            if len(received_subheader) != SUBHEADER_SIZE:
                print('\033[31mWrong subheader size\033[0m')
                sys.exit()


            if message_type == MessageType.PHO_TRAJECTORY_CNT or message_type == MessageType.PHO_TRAJECTORY_FINE:
                if self.response_data.segment_id >= len(
                        self.response_data.trajectory_data):  self.response_data.add_segment()
                waypoints = ()
                waypoint_size = 2 * PACKET_SIZE + NUMBER_OF_JOINTS * PACKET_SIZE
                for iterator in range(payload_size):
                    data = self.client.recv(waypoint_size)
                    waypoint_id = struct.unpack('<i', data[0:4])[0]
                    waypoint = struct.unpack(f'<{NUMBER_OF_JOINTS}f', data[4:(4*NUMBER_OF_JOINTS+4)])
                    check_sum = struct.unpack('<f', data[(4*NUMBER_OF_JOINTS+4):(4*NUMBER_OF_JOINTS+8)])[0]
                    joint_sum = sum(waypoint)
                    # check received joint values
                    if abs(joint_sum - check_sum) > 0.01:
                        print('\033[31mWrong joints sum\033[0m')
                        sys.exit()
                    waypoints = waypoints + waypoint
                    self.response_data.add_waypoint(self.response_data.segment_id,
                                                    waypoint)  # add waypoint to the actual segment of trajectory-
                self.response_data.segment_id += 1  # increment to switch to another segment of trajectory
                self.message = waypoints
                # print data stored in trajectory data
                if self.response_data.print_message == 1: print('\033[94m' + "trajectory: " + '\033[0m' + str(self.response_data.trajectory_data))
                #self.print_message(message_type)
            elif message_type == MessageType.PHO_GRIPPER:
                bytes_to_read = payload_size * PACKET_SIZE
                data = self.client.recv(bytes_to_read)
                self.response_data.gripper_command.append(int(data[0]))  # store gripper command
                self.message = data
                if self.response_data.print_message == 1: print('\033[94m' + "gripper commands: " + '\033[0m' + str(self.response_data.gripper_command))
                #self.print_message(message_type)
            elif message_type == MessageType.PHO_ERROR:
                bytes_to_read = payload_size * PACKET_SIZE  # bytes_to_read = payload_size * PACKET_SIZE
                data = self.client.recv(bytes_to_read)
                error_code = int.from_bytes(data, "little")
                self.message = error_code
                self.response_data.error = error_code
                if self.response_data.print_message == 1: print('\033[94m' + "error message: " + '\033[0m' + str(self.response_data.error))
                #self.print_message(message_type)
            elif message_type == MessageType.PHO_INFO:
                data = self.client.recv(payload_size * PACKET_SIZE)
                self.message = data
                data_size = int((len(data) + 1) / 4)
                info_list = []
                for iterator in range(data_size):
                    info = int.from_bytes(self.message[0 + iterator * PACKET_SIZE:3 + iterator * PACKET_SIZE], "little")
                    info_list.append(info)
                # TRAJECTORY - BPS
                if request_id == ActionRequest.PHO_BINPICKING_TRAJECTORY:
                    self.response_data.gripping_info.append(info_list)
                    if self.response_data.print_message == 1: print('\033[94m' + "gripping info: " + '\033[0m' + str(self.response_data.gripping_info))
                # OBJECT POSE - BPS
                elif request_id == ActionRequest.PHO_BINPICKING_OBJECT_POSE:
                    if object_dimension_flag == 0:
                        self.response_data.dimensions = info_list
                        if self.response_data.print_message == 1: print('\033[94m' + "dimensions: " + '\033[0m' + str(self.response_data.dimensions))
                    elif object_dimension_flag == 1:
                        self.response_data.zheight_angle = info_list
                        if self.response_data.print_message == 1: print('\033[94m' + "z-height/angle: " + '\033[0m' + str(self.response_data.zheight_angle))
                    object_dimension_flag = 1
                # GET VISION SYSTEM STATUS
                elif request_id == ActionRequest.PHO_BINPICKING_GET_VISION_SYSTEM_STATUS:
                    self.response_data.status_data = info_list
                    if self.response_data.print_message == 1: print('\033[94m' + "status data: " + '\033[0m' + str(self.response_data.status_data))
                # GET OBJECTS - LS
                elif request_id == ActionRequest.PHO_LOCATOR_GET_OBJECTS:
                    if object_dimension_flag == 0:
                        self.response_data.dimensions.append(info_list)
                        if self.response_data.print_message == 1: print('\033[94m' + "dimensions: " + '\033[0m' + str(self.response_data.dimensions))
                    elif object_dimension_flag == 1:
                        self.response_data.zheight_angle.append(info_list)
                        if self.response_data.print_message == 1: print('\033[94m' + "z-height/angle: " + '\033[0m' + str(self.response_data.zheight_angle))
                    object_dimension_flag = 1
                elif request_id == ActionRequest.PHO_LOCATOR_GET_VISION_SYSTEM_STATUS:
                    self.response_data.status_data = info_list
                    if self.response_data.print_message == 1: print('\033[94m' + "status data: " + '\033[0m' + str(self.response_data.status_data))
                elif request_id == ActionRequest.PHO_CALIBRATION_SAVE_AUTOMATIC:
                    self.response_data.calib_data = info_list
                    if self.response_data.print_message == 1: print('\033[94m' + "calibration data: " + '\033[0m' + str(self.response_data.calib_data))
                elif request_id == ActionRequest.PHO_SOLUTION_GET_RUNNING:
                    self.response_data.running_solution = info_list
                    if self.response_data.print_message == 1: print('\033[94m' + "running solution: " + '\033[0m' + str(self.response_data.running_solution))
                elif request_id == ActionRequest.PHO_SOLUTION_GET_AVAILABLE:
                    self.response_data.available_solution.append(info_list)
                    if self.response_data.print_message == 1: print('\033[94m' + "available solution: " + '\033[0m' + str(self.response_data.available_solution))
                # self.print_message(message_type)
            elif message_type == MessageType.PHO_OBJECT_POSE:
                data = self.client.recv(payload_size * PACKET_SIZE)
                object_pose = struct.unpack(f'<{CARTES_POSE_LEN}f', data)
                self.message = object_pose
                if request_id == ActionRequest.PHO_CALIBRATION_SAVE_AUTOMATIC:
                    self.response_data.camera_pose = object_pose
                    print('\033[94m' + "camera pose: " + '\033[0m' + str(self.response_data.camera_pose))
                else:
                    self.response_data.object_pose.append(object_pose)
                object_dimension_flag = 0
                # self.print_message(message_type)
            else:
                print('\033[31mUnexpected operation type\033[0m')
                sys.exit()

        # print list of object poses
        if self.response_data.print_message == 1 and self.response_data.object_pose:
            print('\033[94m' + "object pose: "+ '\033[0m' + str(self.response_data.object_pose))

        self.active_request = 0  # request finished - response from request received

    def print_message(self, operation_type):
        if self.print_messages is not True:
            return

        if operation_type == MessageType.PHO_TRAJECTORY_CNT or operation_type == MessageType.PHO_TRAJECTORY_FINE:
            waypoints_size = int((len(self.message) + 1) / NUMBER_OF_JOINTS)
            for x in range(waypoints_size):
                print('\033[94m' + "ROBOT: " + '\033[0m' + "[" + str(
                    round(self.message[x * NUMBER_OF_JOINTS + 0], 2)) + "," + str(
                    round(self.message[x * NUMBER_OF_JOINTS + 1], 2)) + "," + str(
                    round(self.message[x * NUMBER_OF_JOINTS + 2], 2)) + "," + str(
                    round(self.message[x * NUMBER_OF_JOINTS + 3], 2)) + "," + str(
                    round(self.message[x * NUMBER_OF_JOINTS + 4], 2)) + "," + str(
                    round(self.message[x * NUMBER_OF_JOINTS + 5], 2)) + "]")
        elif operation_type == MessageType.PHO_GRIPPER:
            print('\033[94m' + "ROBOT GRIPPER: " + '\033[0m' + "[" + str(self.message[0]) + "]")
        elif operation_type == MessageType.PHO_ERROR:
            print('\033[94m' + "ERROR CODE: " + '\033[0m' + "[" + str(self.message) + "]")
        elif operation_type == MessageType.PHO_INFO:
            data_size = int((len(self.message) + 1) / 4)
            for iterator in range(data_size):
                # check received message size
                if len(self.message) != data_size * PACKET_SIZE:
                    print('\033[31mWrong message size\033[0m')
                    sys.exit()
                info = int.from_bytes(self.message[0 + iterator * PACKET_SIZE:3 + iterator * PACKET_SIZE], "little")
                print('\033[94m' + "INFO: " + '\033[0m' + "[" + str(info) + "]")
        elif operation_type == MessageType.PHO_OBJECT_POSE:
            if CARTES_POSE_LEN == 6:
                print('\033[94m' + "OBJECT: " + '\033[0m' + "[" + str(round(self.message[0], 3)) + "," + str(
                    round(self.message[1], 3)) + "," + str(round(self.message[2], 3)) + "," + str(
                    round(self.message[3], 3)) + "," + str(round(self.message[4], 3)) + "," + str(
                    round(self.message[5], 3)) + "]")
            elif CARTES_POSE_LEN == 7:
                print('\033[94m' + "OBJECT: " + '\033[0m' + "[" + str(round(self.message[0], 3)) + "," + str(
                    round(self.message[1], 3)) + "," + str(round(self.message[2], 3)) + "," + str(
                    round(self.message[3], 3)) + "," + str(round(self.message[4], 3)) + "," + str(
                    round(self.message[5], 3)) + "," + str(round(self.message[6], 3)) + "]")


# -------------------------------------------------------------------
#                     OTHER FUNCTIONS
# -------------------------------------------------------------------

def floatArray2bytes(array):
    msg = []
    for value in array:
        msg = msg + list(struct.pack('<f', value))
    return bytearray(msg)


# -------------------------------------------------------------------
#                      STATE SERVER FUNCTIONS
# -------------------------------------------------------------------

class RobotStateCommunication:
    def __init__(self):
        self.client = None
        self.server = None

    def create_server(self, ROBOT_CONTROLLER_IP, PORT):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((ROBOT_CONTROLLER_IP, PORT))
        # Listen for incoming connections
        self.server.listen(1)
        print('Server is running, waiting for client...')

    def wait_for_client(self):
        self.client, client_address = self.server.accept()
        print('Connection established...')
        # Send hello string
        msg = bytearray(BRAND_IDENTIFICATION_SERVER.encode('utf-8'))
        self.client.send(msg)

    def close_connection(self):
        self.server.close()

    def send_joint_state(self):
        msg = deepcopy(PHO_HEADER)
        msg = msg + struct.pack("ii", NUMBER_OF_JOINTS, JOINT_STATE_TYPE)  # Data size, Type
        msg = msg + floatArray2bytes(get_joint_state(init_joint_state))
        self.client.send(bytearray(msg))

    def send_tool_pose(self):
        msg = deepcopy(PHO_HEADER)
        msg = msg + struct.pack("ii", 7, TOOL_POSE_TYPE)  # Data size, Type
        msg = msg + floatArray2bytes(get_tool_pose(base_quat))
        self.client.send(bytearray(msg))
