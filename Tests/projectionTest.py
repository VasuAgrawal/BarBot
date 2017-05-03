import math
import random

def almostEqual(a, b):
    return abs(a - b) <= 0.0001

numTests = 10

print("Running %d random tests...")

for i in range(numTests):
    testPoint = []
    for j in range(3):
        testPoint.append(random.random()*15)
    rotation = []
    for j in range(3):
        rotation.append(random.random()*6*math.pi - 2*math.pi)
    outputPoint = project(testPoint, rotation)

    for j in range(3):
        if not almostEqual(testPoint[j], outputPoint[j]):
            print("Test %d failed!" % i)
            print("Rotation: %r" % rotation)
            print("TestPoint: %r" % testPoint)
            print("OutputPoint: %r" % outputPoint)
            break

print("Done!")