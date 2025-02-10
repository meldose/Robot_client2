import CommunicationLibrary
import json

CONTROLLER_IP = "192.168.1.1"
PORT = 11003

tool_pose = []

robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

robot.pho_request_solution_start(1)


file_path = 'multiview_pose.json'
#  load JSON data from a file
with open(file_path, 'r') as file:
    json_data = json.load(file)

for point in json_data:
    translation_mm = point["translation"]
    quaternion = point["quaternion"]
    tool_pose = translation_mm + quaternion

    robot.pho_request_locator_trigger_scan(1,tool_pose)
    robot.pho_locator_wait_for_scan()

robot.pho_request_locator_scan(1,tool_pose)
robot.pho_locator_wait_for_scan()


