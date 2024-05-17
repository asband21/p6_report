import ctypes
import socket
import numpy as np
import pickle
import zlib
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
            decompressed_data = zlib.decompress(data)  # Decompress the received data
            scan, cad = pickle.loads(decompressed_data)
        except Exception as e:
            print(f"Failed to deserialize data: {e}")
            conn.close()
            continue
        
        output = np.zeros((4, 4), dtype=np.float64)
        """ 
        cad = np.ascontiguousarray(cad)
        scan = np.ascontiguousarray(scan)
        # Simulate long computation
        print("Starting computation...")
        start_time = time.time()
        print(scan.shape)
        print(cad.shape)
        print(cad[cad.shape[0]-1])
        lokailasiens(
            output.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            scan.shape[0], scan.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            cad.shape[0], cad.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        )
        end_time = time.time()
        print(f"Computation finished in {end_time - start_time} seconds")
        """
        cad = np.ascontiguousarray(cad)
        scan = np.ascontiguousarray(scan)

        # Check if the arrays are contiguous
        assert cad.flags['C_CONTIGUOUS'], "cad array is not contiguous"
        assert scan.flags['C_CONTIGUOUS'], "scan array is not contiguous"

        # Simulate long computation
        print("Starting computation...")
        start_time = time.time()
        print("scan shape:", scan.shape)
        print("cad shape:", cad.shape)
        print("Last element of cad:", cad[-1])

        # Assuming output is another NumPy array for results
        ##output = np.zeros_like(cad)  # or the appropriate shape

        # Call the C++ function via ctypes
        lokailasiens(
            output.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            scan.shape[0], scan.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            cad.shape[0], cad.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))


        result_array = output
        compressed_result = zlib.compress(pickle.dumps(result_array))  # Compress the result before sending
        conn.sendall(compressed_result)
        conn.close()
        print("Result sent back to the client")

if __name__ == "__main__":
    port = 65432  # Replace with the actual port
    receive_arrays_and_send_result(port)

