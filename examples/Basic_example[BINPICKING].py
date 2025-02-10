import CommunicationLibrary

CONTROLLER_IP = "192.168.1.1"
PORT = 11003

start_pose = [0., 0., 0., 0., 0., 0.]
end_pose = [1.5, 0., 0., 0., 0., 0.]


robot = CommunicationLibrary.RobotRequestResponseCommunication()  # object is created
robot.connect_to_server(CONTROLLER_IP, PORT)  # communication between VC and robot is created

robot.pho_request_solution_start(254)
robot.pho_request_binpicking_init(1, start_pose, end_pose)

# request scan
robot.pho_request_binpicking_scan(1)
robot.pho_binpicking_wait_for_scan()

# request trajectory
robot.pho_request_binpicking_trajectory(1)

robot.close_connection()  # communication needs to be closed
