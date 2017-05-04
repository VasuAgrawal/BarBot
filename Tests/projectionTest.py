import math
import random
import numpy as np

class Point(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

def compute_projection(p0, p1, p2):
    # Frame of physical pool
    v1 = np.array([1.0, 0.0, 0.0])
    v2 = np.array([0.0, 1.0, 0.0])
    v3 = np.array([0.0, 0.0, 1.0])

    v = np.transpose(np.vstack((v1, v2, v3)))

    # Frame of global localization points
    w1 = np.array([p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]])
    w1 = w1 / np.linalg.norm(w1)
    w1 = np.array([w1[0][0], w1[1][0], w1[2][0]])
    temp = np.array([p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2]])
    temp = temp / np.linalg.norm(temp)
    temp = np.array([temp[0][0], temp[1][0], temp[2][0]])
    w3 = np.cross(np.transpose(w1), np.transpose(temp))
    w2 = np.cross(w3, w1)

    w = np.transpose(np.vstack((w1, w2, w3)))

    # Use SVD to compute rotation matrix from w to v
    B = np.dot(v, np.transpose(w))
    U, S, V = np.linalg.svd(B)
    M = np.diag([1.0, 1.0, np.linalg.det(U)*np.linalg.det(V)])
    rmat = np.dot(np.dot(U, M), V)

    return rmat

# Projects a point from the GLS frame to the global frame
def project(point, rmat, origin):
    # Subtract origin from the point to account for frame translation
    return rmat.dot(point - origin)

"""
def project(point, rotation):
    # Frame of physical pool
    v1 = np.array([1.0, 0.0, 0.0])
    v2 = np.array([0.0, 1.0, 0.0])
    v3 = np.array([0.0, 0.0, 1.0])

    v = np.transpose(np.vstack((v1, v2, v3)))

    # Frame of global localization points
    #w1 = np.array([p1.x-p0.x, p1.y-p0.y, p1.z-p0.z])
    #w1 = w1 / np.linalg.norm(w1)
    #temp = np.array([p2.x-p0.x, p2.y-p0.y, p2.z-p0.z])
    #temp = temp / np.linalg.norm(temp)
    #w3 = cross(w1, temp)
    #w2 = cross(w3, w1)
    w1 = rotate3D(v1, *rotation)
    w2 = rotate3D(v2, *rotation)
    w3 = rotate3D(v3, *rotation)

    w = np.hstack((w1, w2, w3))

    # Use SVD to compute rotation matrix from w to v
    B = v * np.transpose(w)
    U, S, V = np.linalg.svd(B)
    M = np.diag([1.0, 1.0, np.linalg.det(U)*np.linalg.det(V)])
    rmat = U * M * V

    testRotated = rotate3D(point, *rotation)
    testBack = rmat.dot(testRotated)

    return testBack
"""

def rotate3D(point, x, y, z):
    rmat_x = np.array([[1,0,0], 
              [0, math.cos(x), -math.sin(x)], 
              [0, math.sin(x), math.cos(x)]])
    rmat_y = np.array([[math.cos(y), 0, math.sin(y)],
              [0, 1, 0],
              [-math.sin(y), 0, math.cos(y)]])
    rmat_z = np.array([[math.cos(z), -math.sin(z), 0],
              [math.sin(z), math.cos(z), 0],
              [0, 0, 1]])

    npPoint = np.array([[point[0]], [point[1]], [point[2]]])
    rmat = np.dot(np.dot(rmat_x, rmat_y), rmat_z)

    rotated_point = rmat.dot(npPoint)
    return rotated_point

def almostEqual(a, b):
    return abs(a - b) <= 0.0001

def main2():
    # Calibration points in pool frame
    p0 = [0, 0, 0]
    p1 = [10, 0, 0]
    p2 = [0, 10, 0]

    # Apply transformation to get calibration points into GLS frame
    rotation = [math.pi/4, math.pi/4, math.pi/4]
    translation = np.array([[1.25], [5.23], [6.41]])

    p0_gls = rotate3D(p0, *rotation)
    p1_gls = rotate3D(p1, *rotation)
    p2_gls = rotate3D(p2, *rotation)

    p0_gls += translation
    p1_gls += translation
    p2_gls += translation

    # Compute transform from GLS frame to pool frame
    rmat = compute_projection(p0_gls, p1_gls, p2_gls)

    # Project test point using same rotation into GLS frame
    ptest = [5, 5, 1]
    ptest_gls = rotate3D(ptest, *rotation)
    ptest_gls += translation

    # Apply projection to test point
    ptest_projected = project(ptest_gls, rmat, p0_gls)

    print("Test point in pool frame:")
    print(ptest)
    print("Test point projected back from gls frame:")
    print(ptest_projected)

def main():
    numTests = 1000

    print("Running %d random tests..." % numTests)

    for i in range(numTests):
        testPoint = []
        for j in range(3):
            testPoint.append(random.random()*15)
        rotation = []
        for j in range(3):
            rotation.append(random.random()*math.pi)
        outputPoint = project(testPoint, rotation)

        for j in range(3):
            if not almostEqual(testPoint[j], outputPoint[j]):
                print("Test %d failed!" % i)
                print("Rotation: %r" % rotation)
                print("TestPoint: %r" % testPoint)
                print("OutputPoint: %r" % outputPoint)
                break

    print("Done!")

if (__name__=="__main__"):
    main2()