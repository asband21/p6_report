import pyrealsense2 as rs
import numpy as np
import socket
import pickle
import time
import zlib

per_alo = True

def load_csv_to_numpy(filename):
    try:
        # Load the data from CSV file
        data = np.genfromtxt(filename, delimiter=',')
        return data
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def get_camara_teansformaisen():
    transformation_matrix = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 1],[0, 0, 0, 1]])
    return transformation_matrix

# Initialize the RealSense pipeline

def send_arrays_and_receive_result(array1, array2, receiver_ip, port):
    if array1.shape[1] != 3:
        print("array1 Wrong shape.")
        exit()
    if array2.shape[1] != 3:
        print("array1 Wrong shape.")
        exit()
    if array1.shape[0] < 0 or array1.shape[0] > 100000000:
        print(array1.shape)
        print(array1.shape[0])
        print("array1 have the wrong number of rows.")
        exit()
    if array2.shape[0] < 0 or array2.shape[0] > 100000000:
        print("array2 have the wrong number of rows.")
        exit()
    data = pickle.dumps((array1, array2))                                                                 
    print("comprimer")
    compressed_data = zlib.compress(data)  # Compress the data before sending
    print("sender")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(6000)  # Increase the timeout for the connection
    s.connect((receiver_ip, port))
    s.sendall(compressed_data)
    s.shutdown(socket.SHUT_WR)  # Indicate that we're done sending
    result = b""
    while True:
        try:
            packet = s.recv(4096)
            if not packet:
                break
            result += packet
        except socket.timeout:
            print("Connection timed out")
            break
        except Exception as e:
            print(f"Error receiving data: {e}")
            break
    try:
        decompressed_result = zlib.decompress(result)  # Decompress the received data
        result_array = pickle.loads(decompressed_result)
    except Exception as e:
        print(f"Failed to deserialize result: {e}")
        result_array = None
    s.close()
    return result_array

def filter_points_inside_box(box_min, box_max, points):
    inside_box = np.all((points >= box_min) & (points <= box_max), axis=1)
    filtered_points = points[inside_box]
    return filtered_points

def scan_mod():
    pipeline = rs.pipeline()
    config = rs.config()
    pc = rs.pointcloud()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)

    filename_cad = './gig_AAUTest97_ransed.csv'
    cad = load_csv_to_numpy(filename_cad)

    num_iterations = 50
    if per_alo:
        num_points_per_frame = 640 * 480  # 307200# Pre-allocate an array for all transformed vertices
        all_transformed_vertices = np.empty((num_iterations * num_points_per_frame, 3), dtype=np.float32)
    else:
        all_transformed_vertices = np.empty((0, 3))

    try:
        for i in range(num_iterations):
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
            if per_alo:
                start_index = i * num_points_per_frame
                end_index = start_index + num_points_per_frame
                all_transformed_vertices[start_index:end_index, :] = transformed_vertices
            else:
                all_transformed_vertices = np.vstack((all_transformed_vertices, transformed_vertices))
            #all_transformed_vertices = np.concatenate((all_transformed_vertices, transformed_vertices), axis=1)
            ##print(transformed_vertices)
            print(i)
            print(transformed_vertices.shape)

            time.sleep(0.1)

        #print(all_transformed_vertices)
        #print(all_transformed_vertices.shape)
        boks_min = np.array([0, 0, 0])
        boks_max = np.array([1.1, 1.1, 1.1])
        scan = filter_points_inside_box(boks_min, boks_max, all_transformed_vertices)
        #print("scan")
        #print(scan.shape)
        #print("cad")
        #print(cad)
        #pipeline.stop()
        
        receiver_ip = '100.95.44.35'  # Replace with the actual IP address
        port = 65432  # Replace with the actual port
        result = send_arrays_and_receive_result(cad, scan, receiver_ip, port)
        if result is not None:
            print(result)
        else:
            print("Failed to receive the result")
        return result

    finally:
        pipeline.stop()

if __name__ == "__main__":
    trans = scan_mod()
