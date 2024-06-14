from robolink import * 
from robodk import *  
import time  
import math
import csv 
import keyboard  
from decodeing import Parser
import numpy as np 

RDK = Robolink()  # Create an object RDK to interact with RoboDK 
#setup the station
station_path = "plain_roboDK_simulation_mod.rdk"
station = RDK.AddFile(station_path)  # Load a station
robot = RDK.Item('UR10e', ITEM_TYPE_ROBOT) # if the robot is not provided, the first available robot is used   
robot_base_frame = RDK.Item('UR10eBase', ITEM_TYPE_FRAME) # get the robot base frame 

UR_Data = Parser() # create a UR data parser object


def robot_Mimice_mode():
    # Fetch fresh data
    UR_Data.clear_cache()
    all_data = UR_Data.parse(UR_Data._get_packge())
    while 'JointData' not in all_data or 'CartesianInfo' not in all_data:
        all_data = UR_Data.parse(UR_Data._get_packge()) 
        #time.sleep(0.1)

    joints_in_degrees = [
        math.degrees(all_data['JointData']['q_actual0']),
        math.degrees(all_data['JointData']['q_actual1']),
        math.degrees(all_data['JointData']['q_actual2']),
        math.degrees(all_data['JointData']['q_actual3']),
        math.degrees(all_data['JointData']['q_actual4']),
        math.degrees(all_data['JointData']['q_actual5'])
    ]
    Carensin = [all_data['CartesianInfo']['X']*1000, all_data['CartesianInfo']['Y']*1000, all_data['CartesianInfo']['Z']*1000, all_data['CartesianInfo']['Rx'], all_data['CartesianInfo']['Ry'], all_data['CartesianInfo']['Rz']]
    return  joints_in_degrees,Carensin

def Collision_mapping() : 
    
    target_1 = RDK.Item('target_1',ITEM_TYPE_TARGET)  # get the target from the simulation
    target_2 = RDK.Item('target_2',ITEM_TYPE_TARGET)  # get the target from the simulation 
    target_3 = RDK.Item('target_3',ITEM_TYPE_TARGET)  # get the target from the simulation 
    
    #RDK.PluginCommand("CollisionFreePlanner", "Display", 1) 
    #RDK.Command('CollisionFreePlanner', 'IsUsingWeightedPoints', 0) # set the weighted points to false, using random joints
    status = RDK.PluginCommand("CollisionFreePlanner", "Samples", 100) # number of samples to use for the path generation
    print(status)   

    status = RDK.PluginCommand("CollisionFreePlanner", "Edges",5)# number of edges to use for the path generation
    print(status)

    status = RDK.Command("PathStepCollisionDeg", 4.0) # set the collision step to 4 degrees
    print(status)  

        

    # the name of the program that is going to be generated
    oneTOtwo_name  = "TO_two" 
    twoTOThreee_name = "TO_three"  
    threeTOone_name = "TO_One" 
    # the path generation is started 
    RDK.PluginCommand("CollisionFreePlanner", "Calc") 
    oneTOtwo = RDK.PluginCommand("CollisionFreePlanner", f"Join={oneTOtwo_name}",f"{target_1.Name()}|{ target_2.Name()}") # join the targets ¨
    twoTOthree =RDK.PluginCommand("CollisionFreePlanner", f"Join={twoTOThreee_name}",f"{target_2.Name()}|{ target_3.Name() }")
    threeTOone =RDK.PluginCommand("CollisionFreePlanner", f"Join={threeTOone_name}",f"{target_3.Name()}|{ target_1.Name() }")

    # Will loop until a path is found
    while(oneTOtwo == "Failed" or twoTOthree == "Failed" or threeTOone == "Failed"): 
        print("Path genreation was a  Trying again") 
        RDK.PluginCommand("CollisionFreePlanner", "Calc") 
        #RDK.PluginCommand("CollisionFreePlanner", "Display", 1)  
        
        oneTOtwo = RDK.PluginCommand("CollisionFreePlanner", f"Join={oneTOtwo_name}",f"{target_1.Name()}|{ target_2.Name()}") # join the targets ¨
        twoTOthree =RDK.PluginCommand("CollisionFreePlanner", f"Join={twoTOThreee_name}",f"{target_2.Name()}|{ target_3.Name() }")
        threeTOone =RDK.PluginCommand("CollisionFreePlanner", f"Join={threeTOone_name}",f"{target_3.Name()}|{ target_1.Name() }")

    return oneTOtwo_name, twoTOThreee_name, threeTOone_name

try: 
      
    
    ip = '192.168.0.2'
    port = 30002
    robot.setConnectionParams(ip, port,"/programs/","root","easybot") # set the connection parameters of the robot
    robot.Connect() # Connect to the robot    
    

    #Joints=robot.Joints().list()
    #print(Joints)  
    
    # set robot's joint limits
    #joint_lower_limits = [-165,-180,-165,-200,-270,-270] # set the lower joint limits of the robot
    #joint_upper_limits=[6.8,-61 ,-1,180,270,270] # set the upper joint limits of the robot
    #robot.setJointLimits(joint_lower_limits,joint_upper_limits)# set the joint limits of the robot

    # Set the speed of the robot, simulation and enambe redering
    #robot.setSpeed(1) # Set the speed of the robot to 100 mm/s
    RDK.Render(True) # render the simulation
    #RDK.setSimulationSpeed(0.015) # set the simulation speed t
    #print("Simulations speed : %s " % RDK.SimulationSpeed())   

    #Ur_Joint_values,Carensin_data=robot_Mimice_mode(robot)   
    #print(Ur_Joint_values)
    #robot.setJoints(Ur_Joint_values)  
   


    with open('robodk.csv', mode='w') as file:  
            #robot_pose,robot_joints,robot_ofestet=sample() 
            CVS_MODE = 1
            Points_count =0
            writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            header=['START UP',"robodk AND UR_robot DATA", "Data coordienates are in mm and angles in degress", " "]  
            writer.writerow(header) 
            writer.writerow(["RoboDK"," X-coordinates", "Y-coordinate" ,"Z-coordinate","Rx-coordinates","Ry-coordinates","Rz-coordinates", " Joint 1", "Joint 2" ,"Joint 3","Joint 4","Joint 5","Joint 6", "", "UR_robot"," X-coordinates", "Y-coordinate" ,"Z-coordinate","Rx-coordinates","Ry-coordinates","Rz-coordinates", " Joint 1", "Joint 2" ,"Joint 3","Joint 4","Joint 5","Joint 6"]) 
            
            #writer.writerow([" Joint values", "", robot_joints[0],robot_joints[1],robot_joints[2],robot_joints[3],robot_joints[4],robot_joints[5]]) 
            input("Press Enter to continue...") 

            if False: 
                oneTOtwo_name, twoTOThreee_name, threeTOone_name=Collision_mapping()
                oneTOtwo_program = RDK.Item(oneTOtwo_name,ITEM_TYPE_PROGRAM) # get the program "TO_home" from the simulation
                twoTOThreee_program = RDK.Item(twoTOThreee_name,ITEM_TYPE_PROGRAM) # get the program "TO_weld" from the simulation
                threeTOone_program = RDK.Item(threeTOone_name,ITEM_TYPE_PROGRAM) # get the program "TO_weld" from the simulation
                oneTOtwo_program.setRunType(PROGRAM_RUN_ON_ROBOT) 
                twoTOThreee_program.setRunType(PROGRAM_RUN_ON_ROBOT)  
                threeTOone_program.setRunType(PROGRAM_RUN_ON_ROBOT)
         
            Ur_Joint_values,Carensin_data=robot_Mimice_mode()  
            robot.setJoints(Ur_Joint_values) 
            while True:   
                
                #print(Pose_2_TxyzRxyz(robot.Pose()))
               
                
               
                if CVS_MODE == 0: 
                        Points_count+=1 
                        #print(Carensin_data)
                        print("DATA SVAVED, point %s" % Points_count)  

                        writer.writerow([f"Point {Points_count}", "DATA TYPE" ,"RoboDK Simulation" , " "]) 
                        writer.writerow(["", "X-coordinates", Pose_2_TxyzRxyz(robot.Pose())[0]],)   
                        writer.writerow(["", "Y-coordinates",  Pose_2_TxyzRxyz(robot.Pose())[1]],)   
                        writer.writerow(["", "Z-coordinates",  Pose_2_TxyzRxyz(robot.Pose())[2]],)   
                        writer.writerow(["", "Rx-coordinatess", Pose_2_TxyzRxyz(robot.Pose())[3]],)   
                        writer.writerow(["", "Ry-coordinates",  Pose_2_TxyzRxyz(robot.Pose())[4]],)   
                        writer.writerow(["", "Zz-coordinates",  Pose_2_TxyzRxyz(robot.Pose())[5]],)  
                        writer.writerow(["", "Joint 1",  (np.array(robot.Joints())[0][0])],) 
                        writer.writerow(["", "Joint 2",  (np.array(robot.Joints())[0][1])],) 
                        writer.writerow(["", "Joint 3",  (np.array(robot.Joints())[0][2])],)
                        writer.writerow(["", "Joint 4", (np.array(robot.Joints())[0][3])],)
                        writer.writerow(["", "Joint 5", (np.array(robot.Joints())[0][4])],)
                        writer.writerow(["", (np.array(robot.Joints())[0][5])]) 
                        
                elif CVS_MODE == 1: 


                        input("pioint 1")  
                        #oneTOtwo_program.RunProgram() 
                        #oneTOtwo_program.WaitFinished()
                        
                        Ur_Joint_values,Carensin_data=robot_Mimice_mode()  
                        robot.setJoints(Ur_Joint_values)   
                        print(Ur_Joint_values)
                        time.sleep(1) 
                        print("DATA SVAVED, point %s" % Points_count)  
                        writer.writerow([f"Point1 ,iduration {Points_count}",Pose_2_TxyzRxyz(robot.Pose())[0], Pose_2_TxyzRxyz(robot.Pose())[1], Pose_2_TxyzRxyz(robot.Pose())[2], Pose_2_TxyzRxyz(robot.Pose())[3],Pose_2_TxyzRxyz(robot.Pose())[4], Pose_2_TxyzRxyz(robot.Pose())[5], np.array(robot.Joints())[0][0] ,np.array(robot.Joints())[0][1] ,np.array(robot.Joints())[0][2],np.array(robot.Joints())[0][3],np.array(robot.Joints())[0][4],np.array(robot.Joints())[0][5], "", f"Point {Points_count}", 
                                          Carensin_data[0],Carensin_data[1],Carensin_data[2], Carensin_data[3],Carensin_data[4],Carensin_data[5]  ,Ur_Joint_values[0],Ur_Joint_values[1],Ur_Joint_values[2],Ur_Joint_values[3],Ur_Joint_values[4],Ur_Joint_values[5]])
                         
                        input("pioint 2")  
                        #twoTOThreee_program.RunProgram() 
                        #twoTOThreee_program.WaitFinished() 

                        Ur_Joint_values,Carensin_data=robot_Mimice_mode()  
                        robot.setJoints(Ur_Joint_values)  
                        print(Ur_Joint_values)
                        time.sleep(1) 
                        print("DATA SVAVED, point %s" % Points_count) 
                        writer.writerow([f"Point2 ,iduration {Points_count}",Pose_2_TxyzRxyz(robot.Pose())[0], Pose_2_TxyzRxyz(robot.Pose())[1], Pose_2_TxyzRxyz(robot.Pose())[2], Pose_2_TxyzRxyz(robot.Pose())[3],Pose_2_TxyzRxyz(robot.Pose())[4], Pose_2_TxyzRxyz(robot.Pose())[5], np.array(robot.Joints())[0][0] ,np.array(robot.Joints())[0][1] ,np.array(robot.Joints())[0][2],np.array(robot.Joints())[0][3],np.array(robot.Joints())[0][4],np.array(robot.Joints())[0][5], "", f"Point {Points_count}", 
                                          Carensin_data[0],Carensin_data[1],Carensin_data[2], Carensin_data[3],Carensin_data[4],Carensin_data[5]  ,Ur_Joint_values[0],Ur_Joint_values[1],Ur_Joint_values[2],Ur_Joint_values[3],Ur_Joint_values[4],Ur_Joint_values[5]])
                        
                         
                        input("pioint 3") 
                        #threeTOone_program.RunProgram() 
                        #threeTOone_program.WaitFinished() 

                        Ur_Joint_values,Carensin_data=robot_Mimice_mode()   
                        robot.setJoints(Ur_Joint_values)   
                        print(Ur_Joint_values)
                        time.sleep(1) 
                        print("DATA SVAVED, point %s" % Points_count)
                        writer.writerow([f"Point3 ,iduration {Points_count}",Pose_2_TxyzRxyz(robot.Pose())[0], Pose_2_TxyzRxyz(robot.Pose())[1], Pose_2_TxyzRxyz(robot.Pose())[2], Pose_2_TxyzRxyz(robot.Pose())[3],Pose_2_TxyzRxyz(robot.Pose())[4], Pose_2_TxyzRxyz(robot.Pose())[5], np.array(robot.Joints())[0][0] ,np.array(robot.Joints())[0][1] ,np.array(robot.Joints())[0][2],np.array(robot.Joints())[0][3],np.array(robot.Joints())[0][4],np.array(robot.Joints())[0][5], "", f"Point {Points_count}", 
                                          Carensin_data[0],Carensin_data[1],Carensin_data[2], Carensin_data[3],Carensin_data[4],Carensin_data[5]  ,Ur_Joint_values[0],Ur_Joint_values[1],Ur_Joint_values[2],Ur_Joint_values[3],Ur_Joint_values[4],Ur_Joint_values[5]])
                        
                        Points_count+=1 

                if keyboard.is_pressed('q'):  
                    break
                    
          
finally:  
    robot.Stop()
    robot.Disconnect()
    RDK.CloseRoboDK() 
    print("DONE")