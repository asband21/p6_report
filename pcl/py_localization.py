import socket
import numpy as np
import pickle
import zlib

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
    compressed_data = zlib.compress(data)  # Compress the data before sending
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

def generate_random_array(n):
    return np.random.uniform(-2, 2, (n, 3))

def load_csv_to_numpy(filename):
    try:
        # Load the data from CSV file
        data = np.genfromtxt(filename, delimiter=',')
        return data
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

if __name__ == "__main__":
    #data
    filename_cad = 'reference_pointcloud.csv'
    cad = load_csv_to_numpy(filename_cad)
    cad = generate_random_array(20000000)
    print(cad)
    
    filename_scan = 'scan_RT_pointcloud_w_duplicates_3.csv'
    scan = load_csv_to_numpy(filename_scan)
    print(scan)

    receiver_ip = '100.95.44.35'  # Replace with the actual IP address
    port = 65432  # Replace with the actual port
    
    result = send_arrays_and_receive_result(cad, scan, receiver_ip, port)
    if result is not None:
        print(result)
    else:
        print("Failed to receive the result")

