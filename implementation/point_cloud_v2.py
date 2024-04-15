import pyrealsense2 as rs
import numpy as np

# Function to get the real-world coordinates xy of a point in the image
def point_cloud(coordiantes): 
        
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth)
    config.enable_stream(rs.stream.color)
    pipeline.start(config)

    # Get camera intrinsics
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()

    # Wait for frames
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()

    # Get depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Convert depth frame to numpy array
    depth_image = np.asanyarray(depth_frame.get_data())

    # Calculate real-world distances
    depth_in_meters = depth_image * depth_scale

    # Map pixel coordinates to real-world coordinates
    depth_value = depth_in_meters[coordiantes[1], coordiantes[0]]  # coordinates are in (y, x) format
    Real_world_coordinates = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [coordiantes[0], coordiantes[1]], depth_value)

 
    #print("Real-world coordinates (x, y, z):", Real_world_coordinates)   
    pipeline.stop()  
    return Real_world_coordinates 
    # Release resources

   
    


