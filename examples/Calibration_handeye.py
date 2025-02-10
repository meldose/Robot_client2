import CommunicationLibrary
import json

CONTROLLER_IP = "192.168.1.1"
PORT = 11003

tool_pose = []

robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

robot.pho_request_calibration_start(6,1)
# Load the JSON data
file_path = 'handeye_calib_points.json'
#  load JSON data from a file
with open(file_path, 'r') as file:
    json_data = json.load(file)

# add 9 calibration point
for point in json_data:
    translation_m = point["translation"]
    quaternion = point["quaternion"]
    translation_mm = [x * 1000 for x in translation_m]  # m to mm
    tool_pose = translation_mm + quaternion

    robot.pho_request_calibration_add_point(tool_pose)


robot. pho_request_calibration_save()
robot.pho_request_calibration_stop()