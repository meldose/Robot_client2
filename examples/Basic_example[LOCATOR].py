import CommunicationLibrary

CONTROLLER_IP = "192.168.1.1"
PORT = 11003

robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

robot.pho_request_solution_start(253)

# request scan
robot.pho_request_locator_scan(1)
robot.pho_locator_wait_for_scan()

# request position of located objects
robot.pho_request_locator_get_objects(1, 5)

robot.close_connection()  # communication needs to be closed