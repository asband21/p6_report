import ctypes
import socket
import numpy as np
import pickle
import time

# Load the shared library
lib = ctypes.CDLL('./build/localization.so')  # Use the appropriate path and file name

# Define the function prototype
lokailasiens = lib.lokailasiens
lokailasiens.argtypes = [
    ctypes.POINTER(ctypes.c_double), ctypes.c_int, 
    ctypes.POINTER(ctypes.c_double), ctypes.c_int, 
    ctypes.POINTER(ctypes.c_double)
]
lokailasiens.restype = None

def receive_arrays_and_send_result(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Reuse the address
    s.bind(('0.0.0.0', port))
    s.listen(1)
    print(f"Server listening on port {port}")
    
    while True:
        conn, addr = s.accept()
        print(f"Connection from {addr}")
        data = b""
        while True:
            packet = conn.recv(4096)
            if not packet:
                break
            data += packet
        
        try:
            scan, cad = pickle.loads(data)
        except Exception as e:
            print(f"Failed to deserialize data: {e}")
            conn.close()
            continue
        
        output = np.zeros((4, 4), dtype=np.float64)
        
        # Simulate long computation
        print("Starting computation...")
        start_time = time.time()
        lokailasiens(
            output.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            scan.shape[0], scan.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            cad.shape[0], cad.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        )
        end_time = time.time()
        print(f"Computation finished in {end_time - start_time} seconds")
        
        result_array = output
        result_data = pickle.dumps(result_array)
        conn.sendall(result_data)
        conn.close()
        print("Result sent back to the client")

if __name__ == "__main__":
    port = 65432  # Replace with the actual port
    receive_arrays_and_send_result(port)

