import numpy as np


def get_dist(p1,p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

if __name__ == '__main__':

    print(get_dist((936, 227),(1008, 239)))
    print(get_dist((498, 350),(572, 347)))
    print(get_dist((381, 32),(123, 90)))