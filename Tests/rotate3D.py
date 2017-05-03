import math
import numpy as np

def rotate3D(point, x, y, z):
    rmat_x = np.matrix([[1,0,0], 
              [0, math.cos(x), -math.sin(x)], 
              [0, math.sin(x), math.cos(x)]])
    rmat_y = np.matrix([[math.cos(y), 0, math.sin(y)],
              [0, 1, 0],
              [-math.sin(y), 0, math.cos(y)]])
    rmat_z = np.matrix([[math.cos(z), -math.sin(z), 0],
              [math.sin(z), math.cos(z), 0],
              [0, 0, 1]])

    rotated_point = rmat_x * rmat_y * rmat_z * point
    return rotated_point