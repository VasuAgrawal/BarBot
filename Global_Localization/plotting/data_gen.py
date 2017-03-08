import math
import numpy as np
import matplotlib.pyplot as plt

def noisy_sinusoid(x, sigma2 = .01):
    x = np.array(x)
    return np.sin(x) + np.random.normal(scale=math.sqrt(sigma2), size=x.shape)

x = np.linspace(0, 10 * 2 * math.pi, num = 10 * 25, dtype = float)
data = noisy_sinusoid(x)

with open("mock_data_file", "w") as f:
    f.write("\n".join(map(str, data)))
