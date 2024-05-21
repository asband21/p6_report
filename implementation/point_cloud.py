import pyrealsense2 as rs
import numpy as np

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config() 
pc = rs.pointcloud() 
points = rs.points()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Create point cloud
        
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)

        # Extract point cloud data
        vtx = np.asanyarray(points.get_vertices())
        tex = np.asanyarray(points.get_texture_coordinates())

        # Ensure the texture coordinates have the same number of dimensions as vertices
        tex = np.expand_dims(tex, axis=1)

        # Expand the vertex array to have the same number of dimensions as the texture coordinates
        vtx_expanded = np.expand_dims(vtx, axis=1)

        # Convert dtypes to a common dtype
        vtx_expanded = vtx_expanded.astype(tex.dtype)

        # Combine vertex and color information
        colored_point_cloud = np.concatenate((vtx_expanded, tex), axis=1)


        # Colorize the point cloud
        colorized_point_cloud = np.zeros_like(colored_point_cloud)
        for i in range(len(colorized_point_cloud)):
            u, v = int(tex[i][0] * color_frame.width), int(tex[i][1] * color_frame.height)
            colorized_point_cloud[i, 3:6] = color_image[v, u]

        # Do something with the colorized point cloud data, like visualize it

finally:
    # Stop streaming
    pipeline.stop()
