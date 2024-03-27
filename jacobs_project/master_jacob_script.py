import math
import socket 
import time


def degrees_to_radians(degrees):

    """
    Converts a list of angles from degrees to radians.

    Args:
        degrees: List of angles in degrees.

    Returns:
        List of angles in radians.
    """
    return [math.radians(degree) for degree in degrees]



def send_to(radians):

    """
    Constructs a URScript command to move the robot to a predefined position
    and sends the command to the robot.

    Args:
        robot_ip: Sendts postions to a static IP adress
 
    """ 
    # Static Ip adress of the robot
    robot_ip = '192.168.0.2'

    # Convert the home position to radians
    home_position_radians = degrees_to_radians(radians)
      
    # Convert each radian value to a string
    radian_strings = [str(radian) for radian in home_position_radians]
 
    # Join the radian strings into a comma-separated list
    radian_list_str = ', '.join(radian_strings)

    # Construct the URScript command
    ur_script = "movej([" + radian_list_str + "], a=0.3, v=1.5)\n"  
    
    # Send the URScript command to the robot
    port = 30001

    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as sock:
        sock.connect((robot_ip,port))
        sock.send((ur_script.encode('utf-8')))   
        #sock.send("get_actual_joint_positions()\n ".encode('utf8'))  
        #response =sock.recv(1024).decode()
        #print(response)
    time.sleep(3)     
    print("moving to next")



# Usage  
#Define the home position in degrees  


while(True): 
    position_degrees = [-65.48, -90, -82.66, -98.32, 90, 24.48] 
    send_to(position_degrees)      
    position_degrees = [-40.48, -70, -82.66, -98.32, 90, 24.48]
    send_to(position_degrees)   

# Sectoudn set of postions 


# Look into this next time im over at the Robot
""" 
import sys
import urlib
is_py2 = sys.version[0] == '2'
if is_py2:
   from SimpleXMLRPCServer import SimpleXMLRPCServer
else:
   from xmlrpc.server import SimpleXMLRPCServer

def get_next_pose(p):
    assert type(p) is dict
    pose = urlib.poseToList(p)
    print("Received pose: " + str(pose))
    pose = [-0.18, -0.61, 0.23, 0, 3.12, 0.04];
    return urlib.listToPose(pose);

server = SimpleXMLRPCServer(("", 50000), allow_none=True)
server.RequestHandlerClass.protocol_version = "HTTP/1.1"
print("Listening on port 50000...")

server.register_function(get_next_pose, "get_next_pose")

server.serve_forever()
"""
