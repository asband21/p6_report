#import socket 
import math 
import time 
import urx 
import socket 
from decodeing import ParserUtils


def tcp(command,connection): 
    connection.send(str(command).encode())  
    respons = connection.recv(1024) 
    
    return respons
 
 
def degrees_to_radians(degrees):

    """
    Converts a list of angles from degrees to radians.

    Args:
        degrees: List of angles in degrees.

    Returns:
        List of angles in radians.
    """
    return [math.radians(degree) for degree in degrees]


def send_postion(radians):

    """
    Constructs a URScript command to move the robot to a predefined position
    and sends the command to the robot.

    Args:
        robot_ip: Sendts postions to a static IP adress
 
    """  
    # Convert the home position to radians from degress
    home_position_radians = [math.radians(radian) for radian in radians]
      
    # Convert each radian value to a string
    radian_strings = [str(radian) for radian in home_position_radians]
 
    
    # Join the radian strings into a comma-separated list
    radian_list_str = ', '.join(radian_strings)

    # Construct the URScript command
    ur_script = "movej([" + radian_list_str + "], a=0.3, v=1.5)\n"   
    return ur_script





if __name__ == "__main__":  
    
    # Connection to the UR robot usning TCP commucation 
    ip = '192.168.0.2'   
    port = 30002
    tcp_connection=socket.socket(socket.AF_INET,socket.SOCK_STREAM)   

    tcp_connection.connect((ip,port))         
    #test=ParserUtils
    #print(test.all_data)
    
      
    
    
    target_degree = ([-38.35,-77.38,-101.13,-91.3,90,-38.36])
    target_radian = degrees_to_radians(target_degree)
    
    tcp(send_postion((target_degree),tcp_connection))
    time.sleep(0.3)   
  
    print("target degree = ",target_degree)
    print("target_radian = ",target_radian)
    print("brack")   

        
        
      
        
   
        
    



    
