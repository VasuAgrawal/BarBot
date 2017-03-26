import numpy as np

ref_points = np.array([[1, 1], [-1, -1]], dtype=np.double)

# guess_points = np.ones((2, 2), dtype=np.double)
guess_points = np.array([[0, 0], [1, 0]], dtype=np.double)

def pdist(X):
    # Return a matrix of pairwise distances for rows of X
    num_points = len(X)
    dist = np.zeros((num_points, num_points), dtype=np.double)
    for i in range(num_points):
        for j in range(num_points):
            dist[i][j] = np.linalg.norm(X[j] - X[i])
    return dist

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
    grad_x1 = 2 * (-2 * (M[0][1] - Mhat[0][1]) * (x1 - x2) / np.linalg.norm(x1 - x2))
    grad_x2 = 2 * (-2 * (M[0][1] - Mhat[0][1]) * (x2 - x1) / np.linalg.norm(x1 - x2))
    print("Gradients", grad_x1, grad_x2)
    x1 -= .01 * grad_x1
    x2 -= .01 * grad_x2
    print("x1, x2", x1, x2)
    guess_points = np.array([x1, x2])
    print()

# ref_points = np.array([[0, 0], [1, 0], [0, 1]], dtype=np.double)
ref_points = np.array([[66, 200], [165, 240], [133, 315], [233, 253], [266,
    315], [261, 228], [333, 200], [266, 84], [195, 124], [122, 82], [151, 182]],
    dtype=np.double)
guess_points = np.zeros(ref_points.shape, dtype=np.double)


# Perform random initialization
for i in range(len(ref_points)):
    guess_points[i] = np.random.random(2)
# guess_points = np.array([[0,0], [2,0], [4,4]], dtype=np.double)

def step2():
    global guess_points
    M = pdist(ref_points)
    Mhat = pdist(guess_points)
    print("M", M)
    print("Mhat", Mhat)
    num_points = len(guess_points)

    gradients = np.zeros(guess_points.shape, dtype=np.double)
    for i in range(num_points):
        xi = guess_points[i]
        for j in range(num_points):
            if i == j: continue
            xj = guess_points[j]
            diff = (xi - xj)
            gradients[i] += -2 * (M[i][j] - Mhat[i][j]) * (diff) / np.linalg.norm(diff)
        gradients[i] *= 2 # Correctness
    guess_points -= .01 * gradients
    
    print("Gradients", gradients)
    print()

import matplotlib.pyplot as plt
plt.ion()

def plot():
    xmin = min(ref_points[:, 0].min(), guess_points[:, 0].min())
    ymin = min(ref_points[:, 1].min(), guess_points[:, 1].min())
    xmax = max(ref_points[:, 0].max(), guess_points[:, 0].max())
    ymax = max(ref_points[:, 1].max(), guess_points[:, 1].max())

    ref_wrap = np.vstack((ref_points, ref_points[0]))
    guess_wrap = np.vstack((guess_points, guess_points[0]))

    ref_fig = plt.figure(1)
    plt.clf()
    plt.plot(ref_wrap[:, 0], ref_wrap[:, 1])
    plt.plot(guess_wrap[:, 0], guess_wrap[:, 1])
    plt.xlim((xmin, xmax))
    plt.ylim((ymin, ymax))

    # guess_fig = plt.figure(2)
    # plt.clf()
    # plt.xlim((xmin, xmax))
    # plt.ylim((ymin, ymax))
    plt.show(False)
    plt.pause(.001)
