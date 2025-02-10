import CommunicationLibrary

CONTROLLER_IP = "192.168.1.1"
PORT = 11003

robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

robot.pho_request_solution_start(252)
robot.pho_request_locator_scan(1)
robot.pho_locator_wait_for_scan()
robot.pho_request_locator_get_objects(1, 5)

robot.pho_request_solution_change(253)
robot.pho_request_locator_scan(1)
robot.pho_locator_wait_for_scan()
robot.pho_request_locator_get_objects(1, 5)

robot.close_connection()  # communication needs to be closed
