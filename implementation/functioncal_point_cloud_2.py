import pyrealsense2 as rs
import numpy as np
import time

def get_camara_teansformaisen():
    transformation_matrix = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 1],[0, 0, 0, 1]])
    return transformation_matrix

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
pc = rs.pointcloud()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

pipeline.start(config)

all_transformed_vertices = np.empty((0, 3))

try:
    for i in range(50):
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        points = pc.calculate(depth_frame)
        vertices = np.asarray(points.get_vertices())
        
        vertices = np.asarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        #vertices = np.stack((vertices['f0'], vertices['f1'], vertices['f2']), axis=-1)
        homogeneous_vertices = np.hstack((vertices, np.ones((vertices.shape[0], 1))))
        transformed_homogeneous_vectors = homogeneous_vertices @ get_camara_teansformaisen().T
        transformed_vertices = transformed_homogeneous_vectors[:, :3]
        all_transformed_vertices = np.vstack((all_transformed_vertices, transformed_vertices))
        #all_transformed_vertices = np.concatenate((all_transformed_vertices, transformed_vertices), axis=1)
        ##print(transformed_vertices)
        print(i)
        print(transformed_vertices.shape)

        #time.sleep(1)

    print(all_transformed_vertices)
    print(all_transformed_vertices.shape)
    #pipeline.stop()
finally:
    pipeline.stop()
