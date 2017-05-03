def compute_projection(p0, p1, p2):
    # Frame of physical pool
    v1 = np.array([1.0, 0.0, 0.0])
    v2 = np.array([0.0, 1.0, 0.0])
    v3 = np.array([0.0, 0.0, 1.0])

    v = np.transpose(np.vstack((v1, v2, v3)))

    # Frame of global localization points
    w1 = np.array([p1.x-p0.x, p1.y-p0.y, p1.z-p0.z])
    w1 = w1 / np.linalg.norm(w1)
    temp = np.array([p2.x-p0.x, p2.y-p0.y, p2.z-p0.z])
    temp = temp / np.linalg.norm(temp)
    w3 = cross(w1, temp)
    w2 = cross(w3, w1)

    w = np.transpose(np.vstack((w1, w2, w3)))

    # Use SVD to compute rotation matrix from w to v
    B = dot(v, np.transpose(w))
    U, S, V = np.linalg.svd(B)
    M = np.diag([1.0, 1.0, np.linalg.det(U)*np.linalg.det(V)])
    rmat = U * M * np.transpose(V)

