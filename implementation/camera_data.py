# Import necessary libraries
import pyrealsense2 as rs
import numpy as np   
import cv2 



# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure it to stream
config = rs.config()

# This is the minimal recommended configuration for D435 Depth Camera
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Create an align object


deth_sensor = profile.get_device().first_depth_sensor() 
depth_scale = deth_sensor.get_depth_scale() 
align_to = rs.stream.color
align = rs.align(align_to) 


print(" depth scale : %s" %depth_scale)
# Streaming loop
try:
    while True:
        # Get frameset of color and depth 
        frames = pipeline.wait_for_frames()  
        
        # Align the depth frame to color frame
        aligned_frames =align.process(frames) 

        # Get aligned frames  
        depth_frame = aligned_frames.get_depth_frame()
        color_frame =aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
        depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
        # Validate that both frames are valid 
        
        depth_image = np.array(depth_frame.get_data()) 
        color_image = np.array(color_frame.get_data()) 
        depth = depth_image.astype(float) 
        
        distance = depth_image * depth_scale    
        #depth_with_channels = np.repeat(depth[:, :, np.newaxis], 3, axis=2)   
        
        depth2 = depth_frame.get_distance(int(color_image.shape[0]/2),int(color_image.shape[1]/2))
        #colored_cloud = np.concatenate((color_image, depth_with_channels), axis=1)
        #point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, pixel, depth) 
        print(int(color_image.shape[0]/2))
        print(" distance %s depth2 %s" % (distance[]*100,depth2*100))
        cv2.imshow('RealSense', color_image)
       
       
        
        if  cv2.waitKey(1) & 0xFF == ord('q') or  cv2.waitKey(1) == 27 : 
            cv2.destroyAllWindows()
            break
        

       
        
finally:
    pipeline.stop()