import ctypes
import numpy as np

lib = ctypes.CDLL('./build/libtest.so')
lib.super_fib_list_c.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
lib.super_fib_list_c.restype = None

def call_super_fib_list_c(n):
    output = (ctypes.c_double * (4 * n))()
    lib.super_fib_list_c(output, n)
    return np.array(output).reshape((n, 4))

# Example usage
n = 500
result = call_super_fib_list_c(n)
print("Result:")
print(result)

