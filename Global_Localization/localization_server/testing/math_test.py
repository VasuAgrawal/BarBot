import numpy as np
from scipy.spatial.distance import pdist as scipy_pdist
from scipy.spatial.distance import squareform

ref_points = np.array([[1, 1], [-1, -1]], dtype=np.double)

# guess_points = np.ones((2, 2), dtype=np.double)
# guess_points = np.array([[0, 0], [1, 0]], dtype=np.double)

def norm(x):
    # Apparently writing my own norm is *marginally* faster
    return np.sqrt(np.power(x, 2).sum())

def pdist(X):
    # Unlike the norm, pdist is not a trivial calculation so it works out to be
    # more than 2x as fast to use the scipy pdist rather than using my own.
    return squareform(scipy_pdist(X))

    # # Return a matrix of pairwise distances for rows of X
    # num_points = len(X)
    # dist = np.zeros((num_points, num_points), dtype=np.double)
    # for i in range(num_points):
        # for j in range(num_points):
            # dist[i][j] = norm(X[j] - X[i])
    # return dist

def matrix_loss(A, B):
    # Given two matrices, compute the loss function as a sum over the squared
    # difference of elements
    return np.power(B - A, 2).sum()

# Begin optimization routine somehow ...

# Compute gradient of this matrix somehow?
def step():
    global guess_points
    M = pdist(ref_points)
    Mhat = pdist(guess_points)
    print("M", M)
    print("Mhat", Mhat)
    x1 = guess_points[0]
    x2 = guess_points[1]
    print("x1, x2", x1, x2)
    grad_x1 = 2 * (-2 * (M[0][1] - Mhat[0][1]) * (x1 - x2) / norm(x1 - x2))
    grad_x2 = 2 * (-2 * (M[0][1] - Mhat[0][1]) * (x2 - x1) / norm(x1 - x2))
    print("Gradients", grad_x1, grad_x2)
    x1 -= .01 * grad_x1
    x2 -= .01 * grad_x2
    print("x1, x2", x1, x2)
    guess_points = np.array([x1, x2])
    print()

# Triangle
# ref_points = np.array([[0, 0], [1, 0], [0, 1]], dtype=np.double)
# A spiky crown or something
# ref_points = np.array([[66, 200, 0], [165, 240, 0], [133, 315, 0], 
    # [233, 253, 0], [266, 315, 0], [261, 228, 0], [333, 200, 0], [266, 84, 0],
    # [195, 124, 0], [122, 82, 0], [151, 182, 0]], dtype=np.double)
# A simple cube
ref_points = np.array([[1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, -1],
    [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], dtype=np.double)


# Perform random initialization
# guess_points = np.random.random(guess_points.shape)
# guess_points = np.array([[0,0], [2,0], [4,4]], dtype=np.double)

def step2(guess_points):
    M = pdist(ref_points)
    Mhat = pdist(guess_points)
    # print("M", M)
    # print("Mhat", Mhat)
    num_points = len(guess_points)

    gradients = np.zeros(guess_points.shape, dtype=np.double)
    for i in range(num_points):
        xi = guess_points[i]
        for j in range(num_points):
            if i == j: continue
            xj = guess_points[j]
            diff = (xi - xj)
            gradients[i] += -2 * (M[i][j] - Mhat[i][j]) * (diff) / norm(diff)
        gradients[i] *= 2 # Correctness
    guess_points -= .05 * gradients

    # We want to return the loss, which is the distance between the two matrices
    # M and Mhat.
    return matrix_loss(M, Mhat) 
    # print("Gradients", gradients)
    # print()

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.ion()

def plot(guess_points):
    xmin = min(ref_points[:, 0].min(), guess_points[:, 0].min())
    ymin = min(ref_points[:, 1].min(), guess_points[:, 1].min())
    xmax = max(ref_points[:, 0].max(), guess_points[:, 0].max())
    ymax = max(ref_points[:, 1].max(), guess_points[:, 1].max())

    ref_wrap = np.vstack((ref_points, ref_points[0]))
    guess_wrap = np.vstack((guess_points, guess_points[0]))

    ref_fig = plt.figure(1)
    plt.clf()
    ax = ref_fig.add_subplot(111, projection='3d')
    ax.plot(ref_wrap[:, 0], ref_wrap[:, 1], ref_wrap[:, 2])
    ax.plot(guess_wrap[:, 0], guess_wrap[:, 1], guess_wrap[:, 2])
    ax.set_xlim((xmin, xmax))
    ax.set_ylim((ymin, ymax))

    plt.show(False)
    plt.pause(.001)

def GD():
    # Perform gradient descent
    step_count = 0
    guess_points = np.random.random(ref_points.shape) * .25
    while step_count < 1000:
        loss = step2(guess_points)
        # print("Iteration", step_count, "loss:", loss)
        step_count += 1
        if loss < 1e-10:
            break
    return guess_points, loss, step_count

def test_all(count):
    best_loss = None
    best_points = None
    best_count = None
    for i in range(count):
        guess_points, loss, step_count = GD()
        if best_loss is None or loss < best_loss:
            best_points = guess_points
            best_loss = loss
            best_count = step_count
        # print(loss, step_count)

    # print()
    # print(best_loss, best_count)
    # plot(best_points)

import cProfile
cProfile.run("test_all(100)")
