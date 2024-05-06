"""   
Author: Jakob Gjøderum Jørgensen
This is a test program for the RoboDK API, the program is used to test the connection between the RoboDK API and the RoboDK simulation. 

TODO:  
- Find the posstion of the weld object in the simulation 
- Find the correct weld seam to the weld obejct in the simulation 
-  Make a home position for the robot 
-  Check if the robot can reach the weld object 
-  Make a user operator input , DONE use INPUT FUNCTION

InPROGRESS:  
- Fixiing update fuction so that it orient the tcp to the weld seam for drag or psuh fuction

"""

from robolink import *  # Import the RoboDK API
from robodk import *    # Import the RoboDK API constants 
from robodk.robomath import PosePP as p
import time
# Start RoboDK 
RDK = Robolink()



collisions_bool = False  # set to True to enable collision detection, !!! not nessary for simulation !!!


#set the simulation speed
#RDK.setSimulationSpeed(1) 
def SetUp(): 

    # setup the station
    station_path = "roboDK_simulation.rdk"
    station = RDK.AddFile(station_path)   
    robot = RDK.Item('', ITEM_TYPE_ROBOT) # if the robot is not provided, the first available robot is used  
    path_settings = RDK.Item('Placement(Update)', ITEM_TYPE_MACHINING) # get the path settings from the simulation
    target = RDK.Item('WeldItemFrame',ITEM_TYPE_FRAME)   
    Global_frame = RDK.Item('GlobalFrame',ITEM_TYPE_FRAME) 
    program = RDK.Item('Main',ITEM_TYPE_PROGRAM)  

    #target = RDK.Item()
    tcp = RDK.Item('Torch2',ITEM_TYPE_TOOL)  

    # Set the robot at the global frame , have to tjek if correct
    robot.setPoseFrame(Global_frame) 
    robot.setPoseTool(tcp) 
    robot.setSpeed(0.00000001) # Set the speed of the robot to 100 mm/s
    time.sleep(2) 
    RDK.Render(True) # render the simulation
    RDK.setSimulationSpeed(1) # set the simulation speed to 100% 
    robot.setParam("ShowWorkspace",2) # Work space of the robot given from the TCP 
    # Data from the simulation  
    return robot,path_settings,program,target


def  Update(path_settings,robot,program,target) : 
    Update_pathsettings= {}   
    Update_pathsettings = {
        "Algorithm": 1,  # 0: minimum tool orientation change, 1: tool orientation follows path
        "ApproachRetractAll": 1,
        "AutoUpdate": 1,
        "AvoidCollisions": 0,
        "FollowAngle": 45,
        "FollowAngleOn": 1,
        "FollowRealign": 10,
        "FollowRealignOn": 1,
        "FollowStep": 90,
        "FollowStepOn": 1,
        "JoinCurvesTol": 0.5,
        "OrientXaxis2_X": -2,
        "OrientXaxis2_Y": 90,
        "OrientXaxis2_Z": 2,
        "OrientXaxis_X": 0,
        "OrientXaxis_Y": 0,
        "OrientXaxis_Z": 1,
        "PointApproach": 20,
        "RapidApproachRetract": 1,
        "RotZ_Range": 180,
        "RotZ_Step": 20,
        "SpeedOperation": 50,
        "SpeedRapid": 1000,
        "TrackActive": 1,
        "TrackOffset": 1,
        "TrackVector_X": -2,
        "TrackVector_Y": -2,
        "TrackVector_Z": -2,
        "TurntableActive": 1,
        "TurntableOffset": 1,
        "TurntableRZcomp": 1,
        "VisibleNormals": 1
    }


    """ 
    Update_pathsettings["Algorithm"] = 1 # set the algorithm to 1 for tool to  follows path
    Update_pathsettings['AutoUpdate'] = 0 # set the auto update to false
    Update_pathsettings['SpeedOperation'] = 50 # set the speed of the robot to 100 mm/s  
    Update_pathsettings['OrientXaxis2_Y'] = 90 
    Update_pathsettings['OrientXaxis2_Z'] = 90  
    Update_pathsettings['OrientXaxis2_X'] = 90 
    Update_pathsettings['OrientXaxis_X'] = 90  
    Update_pathsettings['OrientXaxis_Y'] = 90 
    Update_pathsettings['OrientXaxis_Z'] = 90  
    Update_pathsettings["FollowAngleOn"] = 1
    Update_pathsettings["FollowAngle"] = 45 
    """


  

    path_settings.setParam("Machining")
    # get the reachable path of the robot
    new_settings =path_settings.setParam("Machining",Update_pathsettings) # set the speed of the robot to 100 mm/s 
    update_status=path_settings.setParam("UpdatePath")   
    #progevents = path_settings.setParam("ProgEvents")  
    
    # Update the path settings  
    #test=robot.setParam("Machining") # set the collision check to true 
    #target.Update()
    path_settings.Update()
  
    """  
    print(robot.Name())
    print("Simulations speed : %s " % RDK.SimulationSpeed())       

    print(path_settings.setParam("Machining"))
    print("update status: %s "%update_status) # print the speed of the robot 
    #print(status) # print the status of the update 
    print("implemt of new settings are %s" % new_settings)
    """
 


def COLLISION_check(collisions_bool,program): 
    """ 
    I'm not a reglious man, but if you runs this program on a actual robot without enabling the collision detection,  
    may the gods have mercy on your soul
    """
    
    if collisions_bool : # set to True to enable collision detection, !!! not nessary for simulation !!!  
        check_collisions = COLLISION_ON      
    else:    
        check_collisions = COLLISION_OFF

    collions_data=program.Update(check_collisions)
    collions_precentages=collions_data[3]*100 # the collision data in precentages of no collision
    return collions_precentages
    

def Main() :  
    """  
    The main function of the program, this function is used to run the simulation and the program 
    """
    robot,path_settings,program,target=SetUp() # setup the simulation 
    try:   
        robot.setJoints([-90.000000, 0, -150, 0.000000, -90.000000, 0.000000]) 
        
        target.setPose(p(967.629,524.114,15.000,0,0,0.00)) # set the target to new coordinates given from the camera

        input("Press any key to continue...") # wait for the user to press a key to continue the program 
        
        Update(path_settings,robot,program,target) # update the path settings and robot settings
        collions_precentages=COLLISION_check(collisions_bool,program) # check if the program has any collisions
        Update(path_settings,robot,program,target) 
        robot.setSpeed(0.001) # set the speed of the robot to 100 mm/s
        robot.MoveJ(target.Pose()) # move the robot to the home position
     
        
        
        if collions_precentages == 100:  
            print("The program is collision free" )  
            # runing the program "Main"  
            # set the robot joints to zero
            RDK.RunProgram("Main", True) # Run the program "Main" until the end, switch true with False if you want the program stop before the program is finished

        else: 
            print("Warning: The program has a collision risk of %s precentages " % (100-collions_precentages))  
            program.Stop() 

    finally: 
        print("The program is finished") 
        RDK.CloseRoboDK()
Main() 







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
