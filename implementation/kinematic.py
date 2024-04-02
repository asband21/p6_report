import math 

class Kinematic(object): 
    """ 
    This class is used to calculate the Denavit-Hartenberg parameters and the Forward Kinematics for the UR10e robot.
    """   

    def __init__(self):  
        self.Denavit_Hartenberg() 
         
    def Denavit_Hartenberg(self):  
        """  
        This def  is used to calculate the Denavit-Hartenberg parameters for the UR10e robot.  
        theta (list): The joint angles in radians. 
        a (list): The link lengths. 
        d (list): The link offsets. 
        alpha (list): The link twist angles in radians.
        """ 
        self.joints = {"theta": "[rad]" , "r": "[m]", "d": "[m]", "alpha" : "[rad]"} # joints represents the Denavit-Hartenberg parameters and there units
        self.joints["theta"] = [0,0,0,0,0,0] 
        self.joints["r"] = [0, -0.6127, -0.57155, 0, 0, 0] 
        self.joints["d"] = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655] 
        self.joints["alpha"] = [(math.pi)/2, 0, 0, math.pi/2, -math.pi/2, 0] 
        return self.joints
    def Forward_Kinematics(self):  
        """  
        This def is used to calculate the Forward Kinematics for the UR10e robot.  
        theta (list): The joint angles in radians. 
        a (list): The link lengths. 
        d (list): The link offsets. 
        alpha (list): The link twist angles in radians.
        """    
        homogenous_matrix = []  # Define the variable "homogenous_matrix"
        for i in range(6):
           
        
            #make me a 2d array of 4x4 matrix as the momogenous matrix 
            homogenous_matrix *= [[math.cos(self.joints["theta"][i]), -math.sin(self.joints["theta"][i])*math.cos(self.joints["alpha"][i]), math.sin(self.joints["theta"][i])*math.sin(self.joints["alpha"][i]), self.joints["r"][i]*math.cos(self.joints["theta"][i])],
                                 [math.sin(self.joints["theta"][i]), math.cos(self.joints["theta"][i])*math.cos(self.joints["alpha"][i]), -math.cos(self.joints["theta"][i])*math.sin(self.joints["alpha"][i]), self.joints["r"][i]*math.sin(self.joints["theta"][i])],
                                 [0, math.sin(self.joints["alpha"][i]), math.cos(self.joints["alpha"][i]), self.joints["d"][i]],
                                 [0, 0, 0, 1]]
              
              # homogenous_matrix = ([[math.cos(self.joints["theta"][i]),-math.sin(self.joints["theta"][i])*math.cos(self.joints["alpha"][i]),math.sin(self.joints["theta"][i])*math.sin(self.joints["alpha"][i]),self.joints["r"][i]*math.cos(self.joints["theta"][i])], 
              #                  [math.sin(self.joints["theta"][i]),math.cos(self.joints["theta"][i])*math.cos(self.joints["alpha"][i]),-math.cos(self.joints["theta"][i])*math.sin(self.joints["alpha"][i]),self.joints["r"][i]*math.sin(self.joints["theta"][i])], 
              #                  [0,math.sin(self.joints["alpha"][i]),math.cos(self.joints["alpha"][i]),self.joints["d"][i]],[0,0,0,1]])
            
        print(homogenous_matrix)

joints=Kinematic().Forward_Kinematics()
#print(joints)
