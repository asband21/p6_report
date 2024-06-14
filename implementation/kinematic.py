import numpy as np 


def Denavit_Hartenberg():
    # Define Denavit-Hartenberg parameters 
    Denvit_Hartenberg = {"theta": "rad", "r": "m", "d": "m", "alpha": "rad"}
    Denvit_Hartenberg["theta"] = [0, 0, 0, 0, 0, 0]
    Denvit_Hartenberg["r"] = [0, -0.6127, -0.57155, 0, 0, 0]
    Denvit_Hartenberg["d"]= [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]
    Denvit_Hartenberg["alpha"] = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0] 

    return Denvit_Hartenberg


#def transformation(i):
def  transformation(theta,r,d,alpha): 

    # Define transformation matrix  
    """" 
    theta=np.array(Denavit_Hartenberg()["theta"])
    r=np.array(Denavit_Hartenberg()["r"])  
    d=np.array(Denavit_Hartenberg()["d"]) 
    alpha=np.array(Denavit_Hartenberg()["alpha"])   
    """ 
    transformation = [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), r * np.cos(theta)], [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), r * np.sin(alpha)], [0, np.sin(alpha), np.cos(alpha), d], [0, 0, 0, 1]]    
    return transformation.copy()

def Forward_Kinematics(theta): 
    # Calculate Forward Kinematics from base joint to the sixth joint 

    r=np.array(Denavit_Hartenberg()["r"])  
    d=np.array(Denavit_Hartenberg()["d"]) 
    alpha=np.array(Denavit_Hartenberg()["alpha"])  
    #Forward = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(transformation(theta[0], r[0], d[0], alpha[0]), transformation(theta[1], r[1], d[1], alpha[1])), transformation(theta[2], r[2], d[2], alpha[2])), transformation(theta[3], r[3], d[3], alpha[3])), transformation(theta[4], r[4], d[4], alpha[4])), transformation(theta[5], r[5], d[5], alpha[5])) 
    if True:   
        # Define transformation matrix Classic Davit-Hartenberg
        trans0 = [[np.cos(theta[0]), -np.sin(theta[0]) * np.cos(np.pi/2), np.sin(theta[0]) * np.sin(np.pi/2), 0 * np.cos(theta[0])], [np.sin(theta[0]), np.cos(theta[0]) * np.cos(np.pi/2), -np.cos(theta[0]) * np.sin(np.pi/2), 0 * np.sin(np.pi/2)], [0, np.sin(np.pi/2), np.cos(np.pi/2), 0.1807], [0, 0, 0, 1]]    
        trans1 = [[np.cos(theta[1]), -np.sin(theta[1]) * np.cos(0), np.sin(theta[1]) * np.sin(0), -0.6127 * np.cos(theta[1])], [np.sin(theta[1]), np.cos(theta[1]) * np.cos(0), -np.cos(theta[1]) * np.sin(0), -0.6127 * np.sin(0)], [0, np.sin(0), np.cos(0), 0], [0, 0, 0, 1]]
        trans2 = [[np.cos(theta[2]), -np.sin(theta[2]) * np.cos(0), np.sin(theta[2]) * np.sin(0), -0.57155 * np.cos(theta[2])], [np.sin(theta[2]), np.cos(theta[2]) * np.cos(0), -np.cos(theta[2]) * np.sin(0), -0.57155 * np.sin(0)], [0, np.sin(0), np.cos(0), 0], [0, 0, 0, 1]]
        trans3 = [[np.cos(theta[3]), -np.sin(theta[3]) * np.cos(np.pi/2), np.sin(theta[3]) * np.sin(np.pi/2), 0 * np.cos(theta[3])], [np.sin(theta[3]), np.cos(theta[3]) * np.cos(np.pi/2), -np.cos(theta[3]) * np.sin(np.pi/2), 0 * np.sin(np.pi/2)], [0, np.sin(np.pi/2), np.cos(np.pi/2), 0.17415], [0, 0, 0, 1]]    
        trans4 = [[np.cos(theta[4]), -np.sin(theta[4]) * np.cos(-np.pi/2), np.sin(theta[4]) * np.sin(-np.pi/2), 0 * np.cos(theta[4])], [np.sin(theta[4]), np.cos(theta[4]) * np.cos(-np.pi/2), -np.cos(theta[4]) * np.sin(-np.pi/2), 0 * np.sin(-np.pi/2)], [0, np.sin(-np.pi/2), np.cos(-np.pi/2), 0.11985], [0, 0, 0, 1]] 
        trans5 = [[np.cos(theta[5]), -np.sin(theta[5]) * np.cos(0), np.sin(theta[5]) * np.sin(0), 0 * np.cos(theta[5])], [np.sin(theta[5]), np.cos(theta[5]) * np.cos(0), -np.cos(theta[5]) * np.sin(0), 0 * np.sin(0)], [0, np.sin(0), np.cos(0), 0.11655], [0, 0, 0, 1]] 
    else:
        # Define transformation matrix Modified Davit-Hartenberg
        trans0 = [[np.cos(theta[0]),-np.sin(theta[0]),0,r[0-1]], [np.sin(theta[0])*np.cos(alpha[0-1]),np.cos(theta[0])*np.cos(alpha[0-1]),-np.sin(alpha[0-1]),-d[0]*np.sin(alpha[0-1])], [np.sin(theta[0])*np.sin(alpha[0-1]),np.cos(theta[0])*np.sin(alpha[0-1]),np.cos(alpha[0-1]),d[0]*np.cos(alpha[0-1])], [0,0,0,1]]
        trans1 = [[np.cos(theta[1]),-np.sin(theta[1]),0,r[1-1]], [np.sin(theta[1])*np.cos(alpha[1-1]),np.cos(theta[1])*np.cos(alpha[1-1]),-np.sin(alpha[1-1]),-d[1]*np.sin(alpha[1-1])], [np.sin(theta[1])*np.sin(alpha[1-1]),np.cos(theta[1])*np.sin(alpha[1-1]),np.cos(alpha[1-1]),d[1]*np.cos(alpha[1-1])], [0,0,0,1]]
        trans2 = [[np.cos(theta[2]),-np.sin(theta[2]),0,r[2-1]], [np.sin(theta[2])*np.cos(alpha[2-1]),np.cos(theta[2])*np.cos(alpha[2-1]),-np.sin(alpha[2-1]),-d[2]*np.sin(alpha[2-1])], [np.sin(theta[2])*np.sin(alpha[2-1]),np.cos(theta[2])*np.sin(alpha[2-1]),np.cos(alpha[2-1]),d[2]*np.cos(alpha[2-1])], [0,0,0,1]] 
        trans3 = [[np.cos(theta[3]),-np.sin(theta[3]),0,r[3-1]], [np.sin(theta[3])*np.cos(alpha[3-1]),np.cos(theta[3])*np.cos(alpha[3-1]),-np.sin(alpha[3-1]),-d[3]*np.sin(alpha[3-1])], [np.sin(theta[3])*np.sin(alpha[3-1]),np.cos(theta[3])*np.sin(alpha[3-1]),np.cos(alpha[3-1]),d[3]*np.cos(alpha[3-1])], [0,0,0,1]] 
        trans4 = [[np.cos(theta[4]),-np.sin(theta[4]),0,r[4-1]], [np.sin(theta[4])*np.cos(alpha[4-1]),np.cos(theta[4])*np.cos(alpha[4-1]),-np.sin(alpha[4-1]),-d[4]*np.sin(alpha[4-1])], [np.sin(theta[4])*np.sin(alpha[4-1]),np.cos(theta[4])*np.sin(alpha[4-1]),np.cos(alpha[4-1]),d[4]*np.cos(alpha[4-1])], [0,0,0,1]]
        trans5 = [[np.cos(theta[5]),-np.sin(theta[5]),0,r[5-1]], [np.sin(theta[5])*np.cos(alpha[5-1]),np.cos(theta[5])*np.cos(alpha[5-1]),-np.sin(alpha[5-1]),-d[5]*np.sin(alpha[5-1])], [np.sin(theta[5])*np.sin(alpha[5-1]),np.cos(theta[5])*np.sin(alpha[5-1]),np.cos(alpha[5-1]),d[5]*np.cos(alpha[5-1])], [0,0,0,1]]

    if True : 

        trans_0_1 = np.matmul(trans0,trans1) 
        trans_1_2 = np.matmul(trans_0_1,trans2) 
        trans_2_3 = np.matmul(trans_1_2,trans3) 
        trans_3_4 = np.matmul(trans_2_3,trans4) 
        trans_4_5 = np.matmul(trans_3_4,trans5) 
    else: 
        trans_0_1 = np.matmul(trans1, trans0)
        trans_1_2 = np.matmul(trans2,trans_0_1) 
        trans_2_3 = np.matmul(trans3,trans_1_2) 
        trans_3_4 = np.matmul(trans4,trans_2_3) 
        trans_4_5 = np.matmul(trans5,trans_3_4)   
    
    return trans_4_5

  
# Example usage:
if __name__ == '__main__':     

    if False:   
        from decodeing import all_data
        joints=all_data

        kinematic  = Forward_Kinematics([joints['JointData']['q_actual0'],joints['JointData']['q_actual1'],joints['JointData']['q_actual2'],joints['JointData']['q_actual3'],joints['JointData']['q_actual4'],joints['JointData']['q_actual5']])
        print("kinematic \n",kinematic) 
        tool_center_point =  kinematic
        tool_center_point[0:1,3] = tool_center_point[0:1,3]+ 0.00075  
        tool_center_point[0:2,3] = tool_center_point[0:2,3]+ (-0.00249)
        tool_center_point[0:3,3] = tool_center_point[0:3,3]+ 0.43857
        print(all_data['CartesianInfo'])
        print("tcp\n",tool_center_point[0:4]) 
        while False:
            kinematic  = Forward_Kinematics([joints['JointData']['q_actual0'],joints['JointData']['q_actual1'],joints['JointData']['q_actual2'],joints['JointData']['q_actual3'],joints['JointData']['q_actual4'],joints['JointData']['q_actual5']])
            print(kinematic) 
    elif False:  

        theta= np.deg2rad([-61.51,-94.50,-89.05,-86.26,89.91,-57.02])
        print("Theta: \n",theta)
        Kinematic=Forward_Kinematics(theta)   
        
        print("Forward Kinematic: \n",Kinematic) 
        tool_center_point =  Kinematic
        tool_center_point[0:1,3] = tool_center_point[0:1,3]+ 0.00075  
        tool_center_point[0:2,3] = tool_center_point[0:2,3]+ (-0.00249) 
        tool_center_point[0:3,3] = tool_center_point[0:3,3]+ 0.43857 
        print("TCP: \n",tool_center_point)
    else:  
        from decodeing import all_data
        #print(all_data)   
        print(all_data['JointData']['q_actual0'],all_data['JointData']['q_actual1'],all_data['JointData']['q_actual2'],all_data['JointData']['q_actual3'],all_data['JointData']['q_actual4'],all_data['JointData']['q_actual5']) 
        print(all_data['CartesianInfo'])
        # 'CartesianInfo': {'size': 101, 'type': 4, 'X': 0.19835764207055803, 'Y': -0.7328149914118703, 'Z': 0.6418294776661115, 'Rx': 2.125892514876749, 'Ry': -2.3027099828526767, 'Rz': 0.0032229985251860838
        # 'q_actual0': -0.9637444655047815,'q_actual1': -1.464888111954071,  'q_actual2': -1.7108891010284424, 'q_actual3': -1.53567290425811, 'q_actual4': 1.5709656476974487,'q_actual5': -0.9636171499835413
