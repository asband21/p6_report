
import math

import socket



def degrees_to_radians(degrees):

    """

    Converts a list of angles from degrees to radians.



    Args:

        degrees: List of angles in degrees.



    Returns:

        List of angles in radians.

    """

    return [math.radians(degree)
for degree
in degrees]



def send_to_home(robot_ip):

    """

    Constructs a URScript command to move the robot to a predefined home position

    and sends the command to the robot.



    Args:

        robot_ip: IP address of the robot.

    """

    # Define the home position in degrees

    home_position_degrees 
= [-65.48,
-90, 
-82.66, 
-98.32, 
90, 24.48]

    

    # Convert the home position to radians

    home_position_radians 
= degrees_to_radians(home_position_degrees)

    

    # Convert each radian value to a string

    radian_strings 
= [str(radian)
for radian
in home_position_radians]

    

    # Join the radian strings into a comma-separated list

    radian_list_str 
= ', '.join(radian_strings)

    

    # Construct the URScript command

    ur_script 
= "movej([" 
+ radian_list_str 
+ "], a=0.3, v=1.5)\n"

    

    # Send the URScript command to the robot

    port 
= 30002

    with 
socket.socket(socket.AF_INET,
socket.SOCK_STREAM)
as sock:

        sock.connect((robot_ip,
port))

        sock.sendall(ur_script.encode('utf-8'))



# Usage

robot_ip 
= '192.168.0.2'

send_to_home(robot_ip)


