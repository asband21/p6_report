import socket
import numpy as np
import pickle

def send_arrays_and_receive_result(array1, array2, receiver_ip, port):
    data = pickle.dumps((array1, array2))
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(6000)  # Increase the timeout for the connection
    s.connect((receiver_ip, port))
    s.sendall(data)
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
        result_array = pickle.loads(result)
    except Exception as e:
        print(f"Failed to deserialize result: {e}")
        result_array = None

    s.close()
    
    return result_array

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
    filename_scan = 'scan_RT_pointcloud_w_duplicates.csv'
    cad = load_csv_to_numpy(filename_cad)
    scan = load_csv_to_numpy(filename_cad)


    receiver_ip = '127.0.0.1'  # Replace with the actual IP address
    port = 65432  # Replace with the actual port
    
    result = send_arrays_and_receive_result(cad, scan, receiver_ip, port)
    if result is not None:
        print(result)
    else:
        print("Failed to receive the result")
