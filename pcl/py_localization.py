import ctypes

# Load the shared library
lib = ctypes.CDLL('./build/localization.so')  # Use the appropriate path and file name

# Define the function prototype
lokailasiens = lib.lokailasiens
lokailasiens.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
lokailasiens.restype = None

# Example usage
import numpy as np
def load_csv_to_numpy(filename):
    try:
        # Load the data from CSV file
        data = np.genfromtxt(filename, delimiter=',')
        return data
    except Exception as e:
        print(f"An error occurred: {e}")
        return None


filename_cad = 'reference_pointcloud.csv'
filename_scan = 'scan_RT_pointcloud_w_duplicates.csv'
cad = load_csv_to_numpy(filename_cad)
scan = load_csv_to_numpy(filename_cad)
# Assuming you have data for 'scan' and 'cad'
#scan = np.random.rand(100, 3)  # Dummy data
#cad = np.random.rand(100, 3)   # Dummy data
output = np.zeros((4, 4), dtype=np.float64)

# Call the C function
lokailasiens(output.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        scan.shape[0], scan.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        cad.shape[0], cad.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))

print(output)

