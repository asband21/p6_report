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
    return transformation

def Forward_Kinematics(theta): 
    # Calculate Forward Kinematics from base joint to the sixth joint 

    r=np.array(Denavit_Hartenberg()["r"])  
    d=np.array(Denavit_Hartenberg()["d"]) 
    alpha=np.array(Denavit_Hartenberg()["alpha"])  
    Forward = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(transformation(theta[0], r[0], d[0], alpha[0]), transformation(theta[1], r[1], d[1], alpha[1])), transformation(theta[2], r[2], d[2], alpha[2])), transformation(theta[3], r[3], d[3], alpha[3])), transformation(theta[4], r[4], d[4], alpha[4])), transformation(theta[5], r[5], d[5], alpha[5]))

    return Forward


# Example usage:
if __name__ == '__main__':   
    
    kinematic  = Forward_Kinematics([0,0,0,0,0,0]) 
    print(kinematic)
    
    #Kinematic=Forward_Kinematics() 
    #print(Kinematic)
    
    
