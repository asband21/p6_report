import pyrealsense2 as rs
import numpy as np

# Function to get the real-world coordinates xy of a point in the image

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, rs.format.z16, 30) 
color_stream , color_format = rs.stream.color, rs.format.rgb8
config.enable_stream(color_stream, color_format, 30)
pipeline.start(config)

# Get camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w,h = depth_intrinsics.width, depth_intrinsics.height  

# Setup of cloudpoint 
pc = rs.pointcloud() 
decimate = rs.decimation_filter() 
decimate.set_option(rs.option.filter_magnitude, 2) 
colorizer = rs.colorizer() 
filters = [rs.disparity_transform(), rs.spatial_filter(), rs.temporal_filter(), rs.disparity_transform(False)]



# Setup for frames 
color_profil =  rs.video_stream_profile(profile.get_stream(color_stream))
color_intrinsics = color_profil.get_intrinsics() 
color_w, color_h = color_intrinsics.width, color_intrinsics.height 


# Wait for frames 
success,frames = pipeline.try_wait_for_frames(timeout_ms=0)

if not success: 
    print("No frames available") 
    pipeline.stop()


# frame set for color and depth stream
depth_frame = frames.get_depth_frame().as_video_frame()
color_frame = frames.first(color_stream).as_video_frame() 
depth = decimate.process(depth_frame) 

# Gonna grab some new intriniscs
depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

w,h = depth_intrinsics.width, depth_intrinsics.height 

color_image = np.asarray(color_frame.get_data()) 
colorized_depth = colorizer.colorize(depth_frame) 
depth_colormap = np.asanyarray(colorized_depth.get_data()) 

# chanchg to color or depth frame 
if True: 
    mapped_frame, color_soruce = color_frame, color_image 
else: 
    mapped_frame, color_soruce = colorized_depth, depth_colormap


#get points 
points = pc.calculate(depth_frame) 
pc.map_to(mapped_frame)
""" 
Pointcloud data to numpy array  
vtx is the 3D coordinates of the points in the pointcloud 
tex is the texture coordinates of the points in the pointcloud ¨
"""
vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1,3) 
tex = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1,2) 

print(vtx) 
print(tex)


#print("Real-world coordinates (x, y, z):", Real_world_coordinates)   
pipeline.stop()  

# Release resources





