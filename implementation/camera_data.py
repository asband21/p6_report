# Import necessary libraries
import pyrealsense2 as rs
import numpy as np   


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
# Get camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()

# Get camera intrinsics
print(" depth scale : %s" %depth_scale)

# Streaming loop 
def real_world_coordinates(pixels_coordinates):
   
    # Get frameset of color and depth 
    frames = pipeline.wait_for_frames()  
    
    # Align the depth frame to color frame
    aligned_frames =align.process(frames) 

    # Get aligned frames  
    depth_frame = aligned_frames.get_depth_frame()
    color_frame =aligned_frames.get_color_frame()
    
    # Extraxting depth and color images from frames
    depth_image = np.array(depth_frame.get_data()) 
    color_image = np.array(color_frame.get_data()) 
    depth = depth_image.astype(float) 
    
    #center of the camera - coordinates xy 
    center_kamera = pixels_coordinates
    center_kamera = [int(color_image.shape[0]/2), int(color_image.shape[1]/2)]
    
    # two types of depth calculation, in the center of the camera's resolution
    distance = depth_image[center_kamera] * depth_scale    
    distance2 = depth_frame.get_distance(center_kamera[0], center_kamera[1]) 
    
    # Map pixel coordinates to real-world coordinates
    depth_value = distance  # coordinates are in (y, x) format
    Real_world_coordinates = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [center_kamera[0], center_kamera[1]], depth_value)
    print("Real-world coordinates (x, y, z):", Real_world_coordinates) 
    return Real_world_coordinates
            
