# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np


def scatter2d(data):
    transpose = np.array(data).T
    plt.scatter(list(transpose[0]), list(transpose[1]))
    plt.show()
