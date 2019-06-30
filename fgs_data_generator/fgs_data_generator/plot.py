# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np


def scatter2d(data):
    transpose = np.array(data).T
    plt.scatter(list(transpose[0]), list(transpose[1]))
    plt.show()


def model_and_observed(model_data, observed_data, label=None):
    u"""
    Model data : draw as line
    Ovserved data : draw as points
    """
    md_t = np.array(model_data).T
    od_t = np.array(observed_data).T

    if label is not None:
        plt.xlabel(label[0])
        plt.ylabel(label[1])

    plt.plot(list(md_t[0]), list(md_t[1]), label='model')
    plt.scatter(list(od_t[0]), list(od_t[1]), c='red', label='observed')
    plt.legend()

    plt.show()
