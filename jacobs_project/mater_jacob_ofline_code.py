import socket



def send_dashboard_command(robot_ip,command):
    """

    Sends a command to the UR robot's Dashboard Server.



    Args:

        robot_ip: IP address of the robot.

        command: Dashboard Server command as a string.

    """   
    port = 29999  # Dashboard Server port 
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as sock:
        sock.connect((robot_ip,port))
        sock.sendall((command + "\n").encode('utf-8'))  # Commands must be terminated with a newline
        response = sock.recv(1024)  # Read response (optional)
        print(f"Response: {response.decode('utf-8')}")



# Example usage
robot_ip = '192.168.0.2' 

program_name  = 'movewrist_back_forth.urp'


# Load the program
send_dashboard_command(robot_ip,f"load /programs/{program_name}")

# Start execution of the program
send_dashboard_command(robot_ip,"play")
