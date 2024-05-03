import numpy as np

from decodeing import all_data 
from camera_data import real_world_coordinates

def info_extraction() :  
    # Get the joint postion data from the UR Robot, starting from base(q_actual0) to the sixth joint(q_actual5)
    joint_data = all_data['JointData'] 
    joint_postion=joint_data['q_actual0'],joint_data['q_actual1'],joint_data['q_actual2'],joint_data['q_actual3'],joint_data['q_actual4'],joint_data['q_actual5']   
  
    Carensianinfo= all_data['CartesianInfo']  
    # x,y and z coordinates of the tcp (the welding torch) 
    x_tcp,y_tcp,z_tcp,Rx_tcp,Ry_tcp,Rz_tcp= Carensianinfo['X'], Carensianinfo['Y'], Carensianinfo['Z'], Carensianinfo['Rx'], Carensianinfo['Ry'], Carensianinfo['Rz']

    return joint_postion, x_tcp, y_tcp, z_tcp, Rx_tcp, Ry_tcp, Rz_tcp
    
# tranform the  real world coordinates to the robot base frame usning tcp coordinates 
def transformation(x_tcp, y_tcp, z_tcp,Rx_tcp,Ry_tcp,Rz_tcp,real_coordinates): 
    # The transformation matrix from the camera to the robot base frame with orientation 
    #T = np.array([[np.cos(Ry_tcp)*np.cos(Rz_tcp), np.cos(Rz_tcp)*np.sin(Rx_tcp)*np.sin(Ry_tcp)-np.cos(Rx_tcp)*np.sin(Rz_tcp), np.cos(Rx_tcp)*np.cos(Rz_tcp)*np.sin(Ry_tcp)+np.sin(Rx_tcp)*np.sin(Rz_tcp), x_tcp+0.035], [np.cos(Ry_tcp)*np.sin(Rz_tcp), np.cos(Rx_tcp)*np.cos(Rz_tcp)+np.sin(Rx_tcp)*np.sin(Ry_tcp)*np.sin(Rz_tcp), np.cos(Rx_tcp)*np.sin(Ry_tcp)*np.sin(Rz_tcp)-np.cos(Rz_tcp)*np.sin(Rx_tcp), y_tcp+0.105], [-np.sin(Ry_tcp), np.cos(Ry_tcp)*np.sin(Rx_tcp), np.cos(Rx_tcp)*np.cos(Ry_tcp), z_tcp-0.38], [0.0, 0.0, 0.0, 1.0]])
    
    #without orientation
    T = np.array([[1, 0.0, 0.0, x_tcp], [0.0, 1, 0, y_tcp], [0.0, 0.0, 1, z_tcp], [0.0, 0.0, 0.0, 1.0]])  

    # The offset between the camera and the robot base frame
    off_set = np.array([[1,0,0,0.035],[0,1,0,0.105],[0,0,1,-0.38],[0,0,0,1]])  
    # The transformation matrix from the camera to the robot base frame and offset
    T_off = T @ off_set 

    # The real_coordinates_matrix  x and y cooordiantes are flipped because of the camera frame 
    real_coordinates_matrix = np.array([[1,0,0,-real_coordinates[0]],[0,1,0,-real_coordinates[1]],[0,0,1,real_coordinates[2]],[0,0,0,1]])  

    # The transformation matrix from the robot base frame to the camera frame
    T_inv = np.linalg.inv(T_off) 
    # The real world coordinates in the robot base frame
    real_coordinates_robot_base = np.dot(T_inv, real_coordinates_matrix) 
    
    return real_coordinates_robot_base

# From the camera, finds the real world coordiates of the center of the camera  
joint_postion,x_tcp,y_tcp,z_tcp,Rx_tcp,Ry_tcp,Rz_tcp=info_extraction()
real_coordinates = real_world_coordinates([360,620]) # incerte the disred pixel  in x,y coordinates
real_coordinates_robot_base = transformation(x_tcp, y_tcp, z_tcp,Rx_tcp,Ry_tcp,Rz_tcp,real_coordinates) 
print(real_coordinates_robot_base)  