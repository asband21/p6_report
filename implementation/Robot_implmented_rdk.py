"""   
Author: Group 661
This is a test program for the RoboDK API, the program is used to test the connection between the RoboDK API and the RoboDK simulation. 

TODO:  
- Find the posstion of the weld object in the simulation 
- Find the correct weld seam to the weld obejct in the simulation  
-  Make a user operator input 

InPROGRESS:  
- drag mode 
- Mimic mode 
- Free drive mode 
- Get the postions in the robodk simulation 
- Make a start and end point for the weldpath, so the robot knows a path back and foruth
- The weld pathe genreated colites with the weldgun's cable 
Done: 
-  Make a home position for the robot   
- When the collision detection is done, we need to make a path to the object   
- the robot then creates, then it welds 
-when it is done it shall move to the target in which it was left in at the start of the scaning   
- Check if the robot can reach the weld object 




"""
from robolink import *  # Import the RoboDK API
from robodk import *    # Import the RoboDK API constants  
from robodk.robomath import * 
from decodeing import Parser 
from functioncal_point_cloud_2 import weld_item_to_globel_transformaisen
import re  
import math 
import keyboard 
import numpy as np
UR_Data=Parser()
# Start RoboDK 
RDK = Robolink()  
RDK.Command("UseGPU",1) # set the GPU to true, so the simulation uses the GPU for rendering
RDK.Command("AutoRender", 1) # set the auto render to false, so the simulation does not render automatically



collisions_bool = True  # set to True to enable collision detection, !!! not nessary for simulation !!!


def SetUp(): 

    # setup the station
    station_path = "roboDK_simulation_mod.rdk"
    station = RDK.AddFile(station_path)  # Load a station
    robot = RDK.Item('', ITEM_TYPE_ROBOT) # if the robot is not provided, the first available robot is used  
    path_settings = RDK.Item('Placement(Update)', ITEM_TYPE_MACHINING) # get the path settings from the simulation
    target = RDK.Item('WeldItemFrame',ITEM_TYPE_FRAME)  # get the target from the simulation
    camera = RDK.Item('D435_Solid',ITEM_TYPE_OBJECT)  # get the camera from the simulation
    Global_frame = RDK.Item('GlobalFrame',ITEM_TYPE_FRAME)  # get the global frame from the simulation
    
    
    program = RDK.Item('Main',ITEM_TYPE_PROGRAM)  # get the program "Main" from the simulation
   
    # setting the weld tool as the tcp of the robot and the robot to the  global frame
    reffernce=robot.Parent() 
    robot_tcp = robot.Childs() 
    robot.setPoseFrame(reffernce)   
    robot.setPoseTool(robot_tcp[0])   
 
    # set robot's joint limits
    joint_lower_limits = [-165,-180,-165,-200,-270,-270] # set the lower joint limits of the robot
    joint_upper_limits=[6.8,-61 ,-1,180,270,270] # set the upper joint limits of the robot
    robot.setJointLimits(joint_lower_limits,joint_upper_limits)# set the joint limits of the robot
   
    # Set the speed of the robot, simulation and enambe redering
    #robot.setSpeed(1) # Set the speed of the robot to 100 mm/s
    RDK.Render(True) # render the simulation
    RDK.setSimulationSpeed(1) # set the simulation speed t
    print("Simulations speed : %s " % RDK.SimulationSpeed())

    robot.setParam("ShowWorkspace",2) # Work space of the robot given from the TCP 


    return robot,path_settings,program,target,Global_frame,camera
#  -23.000 , 14.000 , 17.300 ,0.000 ,0.000 ,-0.350


def  Update(path_settings,robot,program,target) :  

    # This does not fuction as instened but is used to update the path settings of the robot
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


    # set the path settings of the robot
   # path_settings.setParam("Machining")
    # get the reachable path of the robot
    path_settings.setParam("Machining",Update_pathsettings) # set the speed of the robot to 100 mm/s 
    path_settings.setParam("UpdatePath") 
    
    test=path_settings.Update()  
    status = test[3] # status of the path intagerty
    if status == 0: 
        print("The path may have not been updated yet?")  
    if status == 1:  
        print("The path has been updated") 
    if status == 2: 
        print("The path is not valid")  
       
       
         
     


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


def Collision_mapping(robot_target,weld_targe_external) : 
    
            RDK.PluginCommand("CollisionFreePlanner", "Display", 1) 
            #RDK.Command('CollisionFreePlanner', 'IsUsingWeightedPoints', 0) # set the weighted points to false, using random joints
            status = RDK.PluginCommand("CollisionFreePlanner", "Samples", 20) # number of samples to use for the path generation
            print(status)   

            status = RDK.PluginCommand("CollisionFreePlanner", "Edges",5)# number of edges to use for the path generation
            print(status)

            status = RDK.Command("PathStepCollisionDeg", 4.0) # set the collision step to 4 degrees
            print(status)  

              

            # the name of the program that is going to be generated
            TO_weld_name  = "TO_Weld" 
            TO_home_name = "TO_Home"  
            # the path generation is started 
            RDK.PluginCommand("CollisionFreePlanner", "Calc") 
            status = RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_weld_name}",f"{robot_target.Name()}|{ weld_targe_external.Name()}") # join the targets ¨
            status =RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_home_name}",f"{weld_targe_external.Name()}|{ robot_target.Name() }")
            
           
           # Will loop until a path is found
            while(status== "Failed"): 
                print("Collision_mapping was a %s Trying again"%status) 
                RDK.PluginCommand("CollisionFreePlanner", "Calc") 
                RDK.PluginCommand("CollisionFreePlanner", "Display", 1)  
                
                status = RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_weld_name}",f"{robot_target.Name()}|{ weld_targe_external.Name()}") # join the targets ¨
                status =RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_home_name}",f"{weld_targe_external.Name()}|{ robot_target.Name() }")

            # after the paht is found, the robot is place on the target, but is moved back to its start postion
            
            print("Collision_mapping was a %s"%status) 
            return TO_home_name,TO_weld_name


def robot_Mimice_mode(robot) : 
    
    all_data=UR_Data.parse(UR_Data._get_packge())  
    robot.setJoints([math.degrees(all_data['JointData']['q_actual0']),math.degrees(all_data['JointData']['q_actual1']),math.degrees(all_data['JointData']['q_actual2']),math.degrees(all_data['JointData']['q_actual3']),math.degrees(all_data['JointData']['q_actual4']),math.degrees(all_data['JointData']['q_actual5'])]) # set the joints of the robot to the joints of the program
    

def Scaning_Mode(robot,camera):  
    """  
    Fetches the data from the Decoding.py script and sets the simulations robot's joints to that of the real robot. With the robot correnctly placed the RDK.camera.Pose() is returned
    """ 
    robot_Mimice_mode(robot) # update the robot joints to the real robot joints
    return camera.Pose()
    

def weld_path_selction(robot): 
    """ The user selects the weld seam  with the robot, by recording  the TCP postion in the format of (x,y,z, roll,pitch,yaw)"""  
    robot_Mimice_mode(robot) # update the robot joints to the real robot joints   
    return Pose_2_TxyzRxyz(robot.Pose()) # return the pose of the robot
    

def matrix_to_quaternion(matrix):
    # Ensure the matrix is a numpy array
    matrix = np.array(matrix)
    
    # Extract the 3x3 rotation matrix
    R = matrix[:3, :3]
    
    # Compute the trace of the matrix
    trace = np.trace(R)
    
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    
    # Return the quaternion as a numpy array
    return np.array([qw, qx, qy, qz])


def Main() :  
    """  
    The main function of the program, this function is used to run the simulation and the program  
    """  
    
    try:  
       
        robot,path_settings,program,target,Global_Frame,camera=SetUp() # setup the simulation     
        
        # Mimic the movement of the robots moment for the pointcloud  
        robot_Mimice_mode(robot) # update the robot joints to the real robot joints   
       
       # The camera pose is returned from the scaning mode 

        def replacement_camere_function():
            return (Mat.toNumpy(Scaning_Mode(robot,camera)))
        

        procced_weld_obj_pose=weld_item_to_globel_transformaisen(replacement_camere_function)  
        print(f"The weld object was found at the following pose: {procced_weld_obj_pose}")
        if procced_weld_obj_pose is None: 
            print("The weld object was not found")   
            
        else: 
            qw,qx,qy,qz=matrix_to_quaternion(procced_weld_obj_pose) 
            target.setPose(quaternion_2_pose(qw,qx,qy,qz)) # set the target to new coordinates given from the camera    
        

        TCP_postion=weld_path_selction(robot)
        
        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!# 
        # Then Mads code is used on the location data and the coordiantes are used to select a path for the robot to weld
        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!# 
    
        #This set the target in the simulation, this is where the location code have to inseret a X,Y,Z,Rx,Ry,Rz 
        
        Update(path_settings,robot,program,target) # update the path and robot settings  

        input("Have you switched mode on the UR Robot to remote control?")
        ip = '192.168.0.2'
        port = 30001
        robot.setConnectionParams(ip, port,"/programs/","root","easybot") # set the connection parameters of the robot
        robot.Connect() # Connect to the robot     
      
       
        # Going into drag mode   
        
        # Scan mode 
        
         

        #Accuring the coordiens of the weld pistol (TCP)
       

        
        Update(path_settings,robot,program,target) # update the path settings and robot settings
        # the last update and then the robot is ready to weld 
       
        # set the runMode to one that does not move the real robot
        RDK.setRunMode(4) # int = RUNMODE RUNMODE_SIMULATE=1 performs the simulation moving the robot (default) RUNMODE_QUICKVALIDATE=2 performs a quick check to validate the robot movements RUNMODE_MAKE_ROBOTPROG=3 makes the robot program RUNMODE_RUN_REAL=4 moves the real robot is it is connected
       
        
        
        
        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!# 

        #This instruction number has be the correct instruction number for the weld path, given from Mads code    

        # finds the start point for the weld path, 
        instruction_numb = 5 # the instruction number of the program 

        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#


        # Setting  up target   
        Update(path_settings,robot,program,target)
        Weld_path = RDK.Item('WeldPathExternal',ITEM_TYPE_OBJECT) # get the weld path from the simulation
        robot_target = RDK.AddTarget('robot_target') # get the target from the simulation  
        weld_targe_external = RDK.AddTarget('Weld_target_external')#,Global_Frame)#,target) # making a target for the weld object and making the weldFrame{target} its reffrence frame
        # set the target to the robot start postions
        robot_target.setPose(robot.Pose()) # set the robot target to the robot pose   
    
       
        #the joints from the data are extracted as one whole string and then converted to a list of floats 
        Update(path_settings,robot,program,target)
        pose_str=program.setParam(instruction_numb)["Pose"] 
        tool_path_pose = re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', pose_str)   
        joint_str=program.setParam(instruction_numb)["Joints"] 
        
        tool_path_joint = re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', joint_str)   

        
        # Sets the the weld target to the robots postion
        robot.setJoints([float(tool_path_joint[0]),float(tool_path_joint[1]),float(tool_path_joint[2]),(float(tool_path_joint[3])),(float(tool_path_joint[4])),(float(tool_path_joint[5]))]) # set the joints of the robot to the joints of the program
        weld_targe_external.setPose(robot.Pose())# ser the weld target to the robot pose 

        #returns the robot back to its start postion 
        robot.setJoints(robot_target.Joints()) # set the joints of the robot to the joints of the target
        

        # The program will know being its collision detection and path generation  
        
        TO_home_name,TO_weld_name=Collision_mapping(robot_target,weld_targe_external) # the collision mapping of the robot
        TO_home_program = RDK.Item(TO_home_name,ITEM_TYPE_PROGRAM) # get the program "TO_home" from the simulation
        TO_weld_program = RDK.Item(TO_weld_name,ITEM_TYPE_PROGRAM) # get the program "TO_weld" from the simulation
        TO_home_program.setRunType(PROGRAM_RUN_ON_ROBOT) 
        TO_weld_program.setRunType(PROGRAM_RUN_ON_ROBOT) 
        program.setRunType(PROGRAM_RUN_ON_ROBOT)
        robot.setJoints(robot_target.Joints()) # set the joints of the robot to the joints of the target

        # set the runMode to one that does not move the real robot 
        RDK.setRunMode(RUNMODE_RUN_ROBOT) # int = RUNMODE RUNMODE_SIMULATE=1 performs the simulation moving the robot (default) RUNMODE_QUICKVALIDATE=2 performs a quick check to validate the robot movements RUNMODE_MAKE_ROBOTPROG=3 makes the robot program RUNMODE_RUN_REAL=4 moves the real robot is it is connected
        
        print("RunMode(%s)" % RDK.RunMode())
         
        # Run program 
        Update(path_settings,robot,program,target) 
        
        
        while True: 

            TO_weld_program.RunProgram() # move the robot to the target  
            TO_weld_program.WaitFinished()  
            #program.RunProgram() # Run the program "Main" until the end, switch true with False if you want the program stop before the program is finished 
            #program.WaitFinished()   
            TO_home_program.RunProgram() # move the robot to the target 
            TO_home_program.WaitFinished()  
            
            
           
        """ 
        while True: 
            if TO_weld_program.Busy() == 1: 
                print("Done") 
                TO_home_program.RunProgram()

            if TO_home_program.Busy() == 1: 
                    TO_home_program.RunProgram() 
        """ 

    

        # move the robot to the target
        # A test where the new programs are runned  
        input("wating")
        
        """ while True:    
            print("np") 
            break
            #RDK.RunProgram(TO_weld_name, True) 
            #RDK.RunProgram(TO_home_name, True)   
       """     
            
    
        
        
        if False :
            collions_precentages=COLLISION_check(collisions_bool,program) # check if the program has any collisions 
        
            if collions_precentages == 100:    
                print("The program is collision free" ) 
                #RDK.RunProgram(TO_weld_name, True) # Run the program "Main" until the end, switch true with False if you want the program stop before the program is finished
                
                
                # runing the program "Main"  
                # set the robot joints to zero
                #RDK.RunProgram("Main", True) # Run the program "Main" until the end, switch true with False if you want the program stop before the program is finished
            else: 
                print("Warning: The program has a collision risk of %s precentages " % (100-collions_precentages))  
                program.Stop() 
    


    finally:  
        #input("Press any key to close the program...")
        print("The program is finished")  
        #RDK.Disconnect() 
        robot.Stop
        robot.Disconnect() 
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
