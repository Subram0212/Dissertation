import numpy as np


def main():
    f = lambda x, y: np.square(x - 1) + 10 * (np.square(y - np.square(x)))
    print(f)
    return f
