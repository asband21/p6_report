from robolink import *  # Import the RoboDK API
from robodk import *    # Import the RoboDK API constants
import time
# Start RoboDK 
RDK = Robolink()

station_path = "roboDK_simulation.rdk"
station = RDK.AddFile(station_path)   
robot = RDK.Item('', ITEM_TYPE_ROBOT) 
target = RDK.Item('WeldItem',ITEM_TYPE_TARGET)   
frame = RDK.Item('GlobalFrame',ITEM_TYPE_FRAME)
#target = RDK.Item()
tcp = RDK.Item('Torch2',ITEM_TYPE_TOOL)  
 
robot.setPoseFrame(frame) 
robot.setPoseTool(tcp)
print(robot.Name())
#RDK.ShowRoboDK()  
time.sleep(2)
robot.setJoints([0,0,0,0,0,0]) 
#target_pose = target.Pose()
time.sleep(0.1)
"""     
#Work in progress
robot.setSpeed(50) # mm/s
robot.MoveJ(target)  
time.sleep(0.5)
"""
RDK.CloseRoboDK() # Closese RoboDK -> what did you expect?
#robot.MoveJ(target)    

#print(station.UR10eBase.UR10e.Joints())    
#RDK.CloseRoboDK()




""" 
#Modes  
Mimic_Mode = False

# Start the RoboDK API   

RDK = Robolink() 

Ur_10e = RDK.Item('UR10e')    
Ur_10e.setPoseTool(Ur_10e.PoseTool()) 
robodk.showRoboDK()
# Mimic the movement of the robots moment  
if Mimic_Mode == True:  
    from decodeing import all_data  
    joint_data = all_data['JointData'] 
    success = Ur_10e.Connect()
    Ur_10e.MoveJ([joint_data['q_actual0'], joint_data['q_actual1'], joint_data['q_actual2'], joint_data['q_actual3'], joint_data['q_actual4'], joint_data['q_actual5']])   
"""
