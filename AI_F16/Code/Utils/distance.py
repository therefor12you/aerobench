import numpy as np

def distance(pos_a, pos_b):
    return np.sqrt((pos_a[0] - pos_b[0])**2 + (pos_a[1] - pos_b[1])**2 +(pos_a[2] - pos_b[2])**2)