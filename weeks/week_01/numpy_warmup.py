import numpy as np

A = np.array([[2.0, 0.0], [0.0, 3.0]])
v = np.array([1.0, 1.0])

assert np.allclose(A @ v, np.array([2.0, 3.0]), atol=1e-12)
assert np.allclose(A * v, np.array([[2.0, 0.0], [0.0, 3.0]]), atol=1e-12)

D = np.diag([4.0, 9.0])
eigenvalues, _ = np.linalg.eig(D)
assert np.allclose(np.sort(eigenvalues.real), np.array([4.0, 9.0]), atol=1e-10)

M = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
b = np.array([2, 3, 2])
assert (M + b).shape == (3, 3)
assert np.allclose((M + b)[0, :], M[0, :] + b, atol=1e-10)
