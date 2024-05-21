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
import re   
import keyboard
import math 
import numpy as np

# Start RoboDK 
RDK = Robolink()



collisions_bool = True  # set to True to enable collision detection, !!! not nessary for simulation !!!


def SetUp(): 

    # setup the station
    station_path = "roboDK_simulation.rdk"
    station = RDK.AddFile(station_path)  # Load a station
    robot = RDK.Item('', ITEM_TYPE_ROBOT) # if the robot is not provided, the first available robot is used  
    path_settings = RDK.Item('Placement(Update)', ITEM_TYPE_MACHINING) # get the path settings from the simulation
    target = RDK.Item('WeldItemFrame',ITEM_TYPE_FRAME)  # get the target from the simulation
    Global_frame = RDK.Item('GlobalFrame',ITEM_TYPE_FRAME)  # get the global frame from the simulation
    
    program = RDK.Item('Main',ITEM_TYPE_PROGRAM)  # get the program "Main" from the simulation 
     

    #test=RDK.AddProgram("NewPath",robot)  
    #instruction=LoadList("NewPath.txt")
    #print(instruction)
    
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
    RDK.setSimulationSpeed(0.015) # set the simulation speed t
    print("Simulations speed : %s " % RDK.SimulationSpeed())

    robot.setParam("ShowWorkspace",2) # Work space of the robot given from the TCP 


    return robot,path_settings,program,target,Global_frame


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
    path_settings.setParam("Machining")
    # get the reachable path of the robot
    path_settings.setParam("Machining",Update_pathsettings) # set the speed of the robot to 100 mm/s 
    path_settings.setParam("UpdatePath")   
    path_settings.Update()
  

 


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


def Collision_mapping(robot_target,weld_targe_external, weld_target_exit) : 
    
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
            status_TO_Weld = RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_weld_name}",f"{robot_target.Name()}|{ weld_targe_external.Name()}") # join the targets ¨
            status_TO_Home =RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_home_name}",f"{weld_target_exit.Name()}|{ robot_target.Name() }")
            
           
           # Will loop until a path is found 
            i = 0
            while(status_TO_Weld == "Sucsess" or status_TO_Home == "Sucsess"): 
                print("Path genreation was a %s Trying again"%status) 
                RDK.PluginCommand("CollisionFreePlanner", "Calc") 
                RDK.PluginCommand("CollisionFreePlanner", "Display", 1)  
                
                status_TO_Weld = RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_weld_name}",f"{robot_target.Name()}|{ weld_targe_external.Name()}") # join the targets ¨
                status_TO_Home =RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_home_name}",f"{weld_target_exit.Name()}|{ robot_target.Name() }")
                if i == 2: 
                     break
            # after the paht is found, the robot is place on the target, but is moved back to its start postion
            
            print("Path genreation was a %s"%status) 
            return TO_home_name,TO_weld_name



def Main() :  
    """  
    The main function of the program, this function is used to run the simulation and the program 
    """ 

   
    robot,path_settings,program,target_weldframe,Global_Frame=SetUp() # setup the simulation  
    
    
    try:       
        
        # Going into drag mode   
        
        # Scan mode 
        
        #This set the target in the simulation, this is where the location code have to inseret a X,Y,Z,Rx,Ry,Rz 
        target_weldframe.setPose(TxyzRxyz_2_Pose([967.629,524.114,15.000,0,0,0])) # set the target to new coordinates given from the camera  
       
        #input("ready?")
        Update(path_settings,robot,program,target_weldframe) # update the path and robot settings 
        
        #select the welding seam 
        #Accuring the coordiens of the weld pistol (TCP)
         
        i = 0 
        drag_coordianes = {}
        drag_joints = {}
        while False: 
            drag_coordianes[i]=robot.Pose() 
            drag_joints[i]= robot.Joints()  
            
           
            i = i+1       
        

        #When done selecting, place the robot in any desired postion

        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!# 
        # Then Mads code is used on the location data and the coordiantes are used to select a path for the robot to weld
        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
         
        Update(path_settings,robot,program,target_weldframe) # update the path settings and robot settings
        # the last update and then the robot is ready to weld 
       
        # set the runMode to one that does not move the real robot
        RDK.setRunMode(1) # int = RUNMODE RUNMODE_SIMULATE=1 performs the simulation moving the robot (default) RUNMODE_QUICKVALIDATE=2 performs a quick check to validate the robot movements RUNMODE_MAKE_ROBOTPROG=3 makes the robot program RUNMODE_RUN_REAL=4 moves the real robot is it is connected
       
        
        
        
        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!# 

        #This instruction number has be the correct instruction number for the weld path, given from Mads code    

        # finds the start point for the weld path, 
      

        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#


        # Setting  up target   
        Update(path_settings,robot,program,target_weldframe)
        
       
        #the joints from the data are extracted as one whole string and then converted to a list of floats 
        Update(path_settings,robot,program,target_weldframe) 
      
        
        
        move = {} 
        total_pose= {} 
        xyz_weldpath = {}  
        NewIndices = []
        MinIndex = [] 
        tcp_coords_to_weld = {}
        j =0
                #print(program.InstructionCount()) 
        #input("de er klar til at køre programmet")
        while True: 
            distance = [[],[],[]]
            for i in range(program.InstructionCount()):    
                
                if "MoveJ" in program.setParam(i)['Name'] or "MoveL" in program.setParam(i)['Name']:  
                     
                    move[i]=program.setParam(i)['Pose'] 
                    total_pose[i] = re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', move[i])    
                    xyz_weldpath[i] = total_pose[i][0:3] 
                    
                    tcp_coords_to_weld[j] = Pose_2_TxyzRxyz(robot.Pose())   
                
                    dist = math.sqrt(math.pow(tcp_coords_to_weld[j][0]-float(xyz_weldpath[i][0]), 2) +
                                    math.pow(tcp_coords_to_weld[j][1]-float(xyz_weldpath[i][1]), 2) +
                                    math.pow(tcp_coords_to_weld[j][2]-float(xyz_weldpath[i][2]), 2))  
                    
                    distance[1].append(dist)    
                    distance[2].append(i) 
                    #print(distance[1])
            MinIndex.append(distance[1].index(min(distance[1]))) 
            NewIndices.append(distance[2][MinIndex[j]]) 
               
            j = j+1  
            #print(j)
            if keyboard.is_pressed('q'): 
                break  
    
    
        robot_base = RDK.Item('UR10eBase',ITEM_TYPE_FRAME) # get the base of the robot from the simulation
        Weld_path_matrix_relation_to_target = (robot_base.PoseAbs()).inv() * target_weldframe.PoseAbs() *  Mat.fromNumpy(np.array(
                 [[ 0.000000,     1.000000,    0.000000,   200.000000], 
                 [0.000000,     0.000000,     1.000000,   130.000000], 
                 [1.000000,     0.000000,     0.000000,     0.000000 ], 
                 [0.000000,     0.000000,     0.000000,     1.000000 ]]))



        RDK.AddProgram("ModifiedPath")
        modified_path_program = RDK.Item("ModifiedPath",ITEM_TYPE_PROGRAM) # get the program "Main" from the simulation
        modified_path_program.setRounding(1.0)  
        robot.setPoseFrame(Weld_path_matrix_relation_to_target)
        modified_path_program.setPoseFrame(Weld_path_matrix_relation_to_target) 
        modified_path_program.setSpeed(10000)  
        modified_path_program.setPoseTool(robot.PoseTool()) 

        previous_i = None  
        max_count_of_instructions = 0
        for i in reversed(NewIndices): 
            
            if i != previous_i: 
                    max_count_of_instructions = max_count_of_instructions + 1
                    if "MoveJ" in program.setParam(i)['Name']: 
                        modified_path_program.MoveJ((program.Instruction(i)[5])) 
                    
                    if "MoveL" in program.setParam(i)['Name']: 
                        modified_path_program.MoveL(program.Instruction(i)[4])  
            previous_i = i
        
        
        
       
        instruction_entrance_joints = modified_path_program.Instruction(4)[5]
        instruction_exit = modified_path_program.Instruction(max_count_of_instructions)[5]
        
        robot_target = RDK.AddTarget('robot_target') # get the target from the simulation   
         # set the target to the robot start postions
        robot_target.setPose(robot.Pose()) # set the robot target to the robot pose   
        

        weld_target_entrance = RDK.AddTarget('Weld_target_external')# making a target for the weld object and making the weldFrame{target} its reffrence frame
        robot.setJoints([np.array(instruction_entrance_joints)[0][0],np.array(instruction_entrance_joints)[0][1],np.array(instruction_entrance_joints)[0][2],np.array(instruction_entrance_joints)[0][3],np.array(instruction_entrance_joints)[0][4],np.array(instruction_entrance_joints)[0][5]]) # set the joints of the robot to the joints of the program
        weld_target_entrance.setPose(robot.Pose())# ser the weld target to the robot pose  


        weld_target_exit = RDK.AddTarget('Weld_target_exit') # making a target for the weld object and making the weldFrame{target} its reffrence frame
        robot.setJoints([np.array(instruction_exit)[0][0],np.array(instruction_exit)[0][1],np.array(instruction_exit)[0][2],np.array(instruction_exit)[0][3],np.array(instruction_exit)[0][4],np.array(instruction_exit)[0][5]]) # set the joints of the robot to the joints of the program
        weld_target_exit.setPose(robot.Pose())# ser the weld target to the robot pose 


       
        # The program will know being its collision detection and path generation  
        
        TO_home_name,TO_weld_name=Collision_mapping(robot_target,weld_target_entrance,weld_target_exit) # the collision mapping of the robot
        robot.setJoints(robot_target.Joints()) # set the joints of the robot to the joints of the target

        # set the runMode to one that does not move the real robot 
        RDK.setRunMode(1) # int = RUNMODE RUNMODE_SIMULATE=1 performs the simulation moving the robot (default) RUNMODE_QUICKVALIDATE=2 performs a quick check to validate the robot movements RUNMODE_MAKE_ROBOTPROG=3 makes the robot program RUNMODE_RUN_REAL=4 moves the real robot is it is connected
        
        print("RunMode(%s)" % RDK.RunMode())
         
        # Run program 
        Update(path_settings,robot,program,target_weldframe)
        TO_home_program = RDK.Item(TO_home_name,ITEM_TYPE_PROGRAM) # get the program "TO_home" from the simulation
        TO_weld_program = RDK.Item(TO_weld_name,ITEM_TYPE_PROGRAM) # get the program "TO_weld" from the simulation
        # A test where the new programs are runned  
        
        while True: 

            TO_weld_program.RunProgram() # move the robot to the target  
            TO_weld_program.WaitFinished()  
            modified_path_program.RunProgram() # Run the program "Main" until the end, switch true with False if you want the program stop before the program is finished 
            program.WaitFinished()   
            TO_home_program.RunProgram() # move the robot to the target 
            TO_home_program.WaitFinished()  
            
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
        input("Press any key to close the program...")
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
