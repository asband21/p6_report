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
#from PathSnapping import path_snipping
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
    station_path = "./roboDK_simulation_mod.rdk"
    station = RDK.AddFile(station_path)  # Load a station
    robot = RDK.Item('', ITEM_TYPE_ROBOT) # if the robot is not provided, the first available robot is used  
    path_settings = RDK.Item('Placement(Update)', ITEM_TYPE_MACHINING) # get the path settings from the simulation
    weld_frame = RDK.Item('WeldItemFrame',ITEM_TYPE_FRAME)  # get the target from the simulation
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


    return robot,path_settings,program,weld_frame,Global_frame,camera
#  -23.000 , 14.000 , 17.300 ,0.000 ,0.000 ,-0.350



def  Update(path_settings) :  

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
        status_srt = "The path may have not been updated yet?"
        print("The path may have not been updated yet?")  
    if status == 1:   
        status_srt = "The path has been updated"
        print("The path has been updated")  

    if status == 2:  
        status_srt = "The path is not valid"
        print("The path is not valid")   
    return status_srt
       
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


def Collision_mapping(robot_target,weld_target_entrance,weld_target_exit) : 
    
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
            status_TO_Weld = RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_weld_name}",f"{robot_target.Name()}|{ weld_target_entrance.Name()}") # join the targets ¨
            status_TO_Home =RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_home_name}",f"{weld_target_exit.Name()}|{ robot_target.Name() }")
            

           # Will loop until a path is found
            while(status_TO_Weld != "Failed" and status_TO_Home != "Failed"): 
                print("Path genreation was a %s Trying again"%status) 
                RDK.PluginCommand("CollisionFreePlanner", "Calc") 
                RDK.PluginCommand("CollisionFreePlanner", "Display", 1)  
                
                status_TO_Weld = RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_weld_name}",f"{robot_target.Name()}|{ weld_target_entrance.Name()}") # join the targets ¨
                status_TO_Home =RDK.PluginCommand("CollisionFreePlanner", f"Join={TO_home_name}",f"{weld_target_exit.Name()}|{ robot_target.Name() }")
                
            # after the paht is found, the robot is place on the target, but is moved back to its start postion
            
            print("Collision_mapping was:  HOME: %s, WELD:%s "% (status_TO_Home,status_TO_Weld ) ) 
            return TO_home_name,TO_weld_name

"""
def robot_Mimice_mode(robot) : 
    
    all_data=UR_Data.parse(UR_Data._get_packge())  
    robot.setJoints([math.degrees(all_data['JointData']['q_actual0']),math.degrees(all_data['JointData']['q_actual1']),math.degrees(all_data['JointData']['q_actual2']),math.degrees(all_data['JointData']['q_actual3']),math.degrees(all_data['JointData']['q_actual4']),math.degrees(all_data['JointData']['q_actual5'])]) # set the joints of the robot to the joints of the program
"""
def robot_Mimice_mode(robot):
    # Fetch fresh data
    UR_Data.clear_cache()
    all_data = UR_Data.parse(UR_Data._get_packge())
    while 'JointData' not in all_data:
        all_data = UR_Data.parse(UR_Data._get_packge())

    #Convert joint data from radians to degrees
    joints_in_degrees = [
        math.degrees(all_data['JointData']['q_actual0']),
        math.degrees(all_data['JointData']['q_actual1']),
        math.degrees(all_data['JointData']['q_actual2']),
        math.degrees(all_data['JointData']['q_actual3']),
        math.degrees(all_data['JointData']['q_actual4']),
        math.degrees(all_data['JointData']['q_actual5'])
    ]
    print(joints_in_degrees)
    robot.setJoints(joints_in_degrees)


def robot_Mimice_mode_2(robot):
    # Fetch fresh data
    UR_Data.clear_cache()
    all_data = UR_Data.parse(UR_Data._get_packge())
    while 'JointData' not in all_data:
        all_data = UR_Data.parse(UR_Data._get_packge())

    #Convert joint data from radians to degrees
    joints_in_degrees = [
        math.degrees(all_data['JointData']['q_actual0']),
        math.degrees(all_data['JointData']['q_actual1']),
        math.degrees(all_data['JointData']['q_actual2']),
        math.degrees(all_data['JointData']['q_actual3']),
        math.degrees(all_data['JointData']['q_actual4']),
        math.degrees(all_data['JointData']['q_actual5'])
    ]
    print(joints_in_degrees)
    robot.setJoints(joints_in_degrees)


def Scaning_Mode(robot,camera):  
    """  
    Fetches the data from the Decoding.py script and sets the simulations robot's joints to that of the real robot. With the robot correnctly placed the RDK.camera.Pose() is returned
    """ 
    robot_Mimice_mode(robot) # update the robot joints to the real robot joints
    return camera.PoseAbs()
    

def weld_path_selction(robot,program,target_weldframe): 
    """ The user selects the weld seam  with the robot, by recording  the TCP postion  \n
        ||The function will return the instruction number of the start and end of the weld path and the new modified_welding_path_program|| 
    """  
    move = {} 
    total_pose= {} 
    xyz_weldpath = {}  
    NewIndices = []
    MinIndex = [] 
    tcp_coords_to_weld = {}
    j =0
             
    input("Is the robot set to record th TCP points, if yes press any buttion to continue") 
    print("Press 'q' to stop the program")
    while True:  
        # record the TCP postion of the robot and which there is closed to the original weld path 
        
        robot_Mimice_mode_2(robot) # update the robot joints to the real robot joints

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
    weld_path = RDK.Item('WeldPath',ITEM_TYPE_FRAME) # get the program "Main" from the simulation 
    #weld_path.AddFrame("weld_path",target_weldframe) # set the pose of the weld path to the target pose 
    weld_path.setPoseFrame(Weld_path_matrix_relation_to_target) # set the pose of the weld path to the target pose



    RDK.AddProgram("ModifiedPath") 
    modified_path_program = RDK.Item("ModifiedPath",ITEM_TYPE_PROGRAM) # get the program "Main" from the simulation
    modified_path_program.setRounding(1.0)  
    modified_path_program.setPoseFrame(Weld_path_matrix_relation_to_target) 
    modified_path_program.setSpeed(10000)  
    modified_path_program.setPoseTool(robot.PoseTool()) 

    previous_i = None  
    max_count_of_instructions = 0
    for i in reversed(NewIndices): 
        
        if i != previous_i: 
                if "MoveJ" in program.setParam(i)['Name']: 
                    modified_path_program.MoveJ((program.Instruction(i)[5]))  
                    max_count_of_instructions = max_count_of_instructions +1 
                
                if "MoveL" in program.setParam(i)['Name']: 
                    modified_path_program.MoveL(program.Instruction(i)[4])   
                    max_count_of_instructions = max_count_of_instructions +1 
        previous_i = i 
    
    instruction_entrance_joints = modified_path_program.Instruction(4)[5]
    instruction_exit_joints = modified_path_program.Instruction(modified_path_program.InstructionCount()-1)[5]   
 
    print(max_count_of_instructions)
    print(instruction_exit_joints) 
    print(modified_path_program.InstructionCount())
    return instruction_entrance_joints,instruction_exit_joints, modified_path_program,weld_path

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
       
        robot,path_settings,program,target_weldframe,Global_Frame,camera=SetUp() # setup the simulation     
        
        # Mimic the movement of the robots moment for the pointcloud  
        robot_Mimice_mode(robot) # update the robot joints to the real robot joints   
       
       
        
        

        # The camera pose is returned from the scaning mode 
        input("Are you ready?")
        def replacement_camere_function():
            #robot_Mimice_mode(robot)
            cam = Scaning_Mode(robot,camera)
            #Global_frame = RDK.Item('GlobalFrame', ITEM_TYPE_FRAME)
            ##cam = Global_frame.PoseRel(camera)
            pos = Mat.toNumpy(cam.inv() * Global_Frame.PoseAbs())
            print(pos)
            pos[0, 3] = pos[0, 3]/1000
            pos[1, 3] = pos[1, 3]/1000
            pos[2, 3] = pos[2, 3]/1000
            return pos

        procced_weld_obj_pose=weld_item_to_globel_transformaisen(replacement_camere_function)  
        time.sleep(0.05)
        print(f"The weld object was found at the following pose: {procced_weld_obj_pose}")
        if procced_weld_obj_pose is None: 
            print("The weld object was not found")   
        else: 
            qw,qx,qy,qz=matrix_to_quaternion(procced_weld_obj_pose) 
            target_weldframe.setPose(quaternion_2_pose(qw,qx,qy,qz)) # set the target to new coordinates given from the camera    
        
        
        # The weld path is selected by the user, the user selects the weld seam by recording the TCP postion in the format of (x,y,z, roll,pitch,yaw)  

        Update(path_settings) # update the path and robot settings
        instruction_entrance_joints,instruction_exit_joints,modified_path_program,Weld_path_matrix_relation_to_target=weld_path_selction(robot,program,target_weldframe)
        print(instruction_entrance_joints) 
        print("+++++++++++++++++++++++++++++++++")
        print(instruction_exit_joints) 

        Update(path_settings) # update the path and robot settings  
    
        input("Have you switched mode on the UR Robot to remote control?")
        ip = '192.168.0.2'
        port = 30002
        robot.setConnectionParams(ip, port,"/programs/","root","easybot") # set the connection parameters of the robot
        robot.Connect() # Connect to the robot     
      
        # set the runMode to one that does not move the real robot
        RDK.setRunMode(4) # int = RUNMODE RUNMODE_SIMULATE=1 performs the simulation moving the robot (default) RUNMODE_QUICKVALIDATE=2 performs a quick check to validate the robot movements RUNMODE_MAKE_ROBOTPROG=3 makes the robot program RUNMODE_RUN_REAL=4 moves the real robot is it is connected
       

        robot_target = RDK.AddTarget('robot_target') # get the target from the simulation   
         # set the target to the robot start postions
        robot_target.setPose(robot.Pose()) # set the robot target to the robot pose   
    
        weld_target_entrance = RDK.AddTarget('weld_target_entrance_lllll')#,Weld_path_matrix_relation_to_target)# making a target for the weld object and making the weldFrame{target} its reffrence frame
        gg = robot.setJoints([np.array(instruction_entrance_joints)[0][0],np.array(instruction_entrance_joints)[0][1],np.array(instruction_entrance_joints)[0][2],np.array(instruction_entrance_joints)[0][3],np.array(instruction_entrance_joints)[0][4],np.array(instruction_entrance_joints)[0][5]]) # set the joints of the robot to the joints of the program
        weld_target_entrance.setPoseAbs(RDK.Item('Torch').PoseAbs())# ser the weld target to the robot pose
        #weld_target_entrance.setPoseAbs((robot.Childs()[0]).PoseAbs())# ser the weld target to the robot pose

        print(robot.Pose())

        weld_target_exit = RDK.AddTarget('Weld_target_exit_kkhgfg')# Weld_path_matrix_relation_to_target) # making a target for the weld object and making the weldFrame{target} its reffrence frame
        robot.setJoints([np.array(instruction_exit_joints)[0][0],np.array(instruction_exit_joints)[0][1],np.array(instruction_exit_joints)[0][2],np.array(instruction_exit_joints)[0][3],np.array(instruction_exit_joints)[0][4],np.array(instruction_exit_joints)[0][5]]) # set the joints of the robot to the joints of the program
        weld_target_exit.setPoseAbs(RDK.Item('Torch').PoseAbs())# ser the weld target to the robot pose 
        print(RDK.Itme('Torch').TCP().PoseAbs())

        # The program will know being its collision detection and path generation   

        TO_home_name,TO_weld_name=Collision_mapping(robot_target,weld_target_entrance,weld_target_exit) # the collision mapping of the robot
        TO_home_program = RDK.Item(TO_home_name,ITEM_TYPE_PROGRAM) # get the program "TO_home" from the simulation
        TO_weld_program = RDK.Item(TO_weld_name,ITEM_TYPE_PROGRAM) # get the program "TO_weld" from the simulation
        TO_home_program.setRunType(PROGRAM_RUN_ON_ROBOT) 
        TO_weld_program.setRunType(PROGRAM_RUN_ON_ROBOT)  
        modified_path_program.setRunType(PROGRAM_RUN_ON_ROBOT)
        robot.setJoints(robot_target.Joints()) # set the joints of the robot to the joints of the target

        # set the runMode to one that does not move the real robot 
        RDK.setRunMode(RUNMODE_RUN_ROBOT) # int = RUNMODE RUNMODE_SIMULATE=1 performs the simulation moving the robot (default) RUNMODE_QUICKVALIDATE=2 performs a quick check to validate the robot movements RUNMODE_MAKE_ROBOTPROG=3 makes the robot program RUNMODE_RUN_REAL=4 moves the real robot is it is connected
        
        print("RunMode(%s)" % RDK.RunMode())
         
        # Run program 
        Update(path_settings) 
        
       
        while True: 

            TO_weld_program.RunProgram() # move the robot to the target  
            TO_weld_program.WaitFinished()  
            modified_path_program.RunProgram() # move the robot to the target 
            modified_path_program.WaitFinished()
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
        input("Press any key to close the program...")
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
