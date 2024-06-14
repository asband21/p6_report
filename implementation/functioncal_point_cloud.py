import pyrealsense2 as rs
import numpy as np

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config() 
pc = rs.pointcloud()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

pipeline.start(config)

if __name__ == "__main__": 
    import open3d as o3d
    # Create Open3D visualization window
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window()

    # Enable zooming using keyboard controls
    visualizer.get_view_control().change_field_of_view(step=-1)  # Set initial field of view
    visualizer.get_render_option().point_size = 1.0  # Set initial point size

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue
        
        # Create point cloud from depth frame
        points = pc.calculate(depth_frame)

        # Extract XYZ coordinates from the vertices
        vertices = np.asarray(points.get_vertices())
        points_xyz = np.stack((vertices['f0'], vertices['f1'], vertices['f2']), axis=-1)
       
       

        if __name__ == "__main__": 
             # Convert point cloud to numpy array with float32 data type
            points_xyz = points_xyz.astype(np.float32) 
            # Convert point cloud to Open3D format
            pcd = o3d.geometry.PointCloud() 
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            pcd.points = o3d.utility.Vector3dVector(points_xyz)

            # Set colors from color frame
            color_image = np.asanyarray(color_frame.get_data())
            pcd.colors = o3d.utility.Vector3dVector(color_image.reshape(-1, 3) / 255.0)

            # Update visualization
            visualizer.clear_geometries()
            visualizer.add_geometry(pcd)
            visualizer.poll_events()
            visualizer.update_renderer() 


finally:
    pipeline.stop()
    visualizer.destroy_window()


