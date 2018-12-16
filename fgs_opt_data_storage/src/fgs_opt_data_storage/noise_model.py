# -*- coding: utf-8 -*-
import numpy as np


def create(conf_dict):
    if conf_dict['type'] == 'gaussian':
        sigma = conf_dict['sigma']
        return Gaussian(sigma)
    else:
        raise NotImplementedError(
                '{} is not implemented.'.format(conf_dict['type']))


class Gaussian():
    def __init__(self, sigma):
        if sigma <= 0:
            raise Exception('Invalid sigma of gaussian')

        self.__sigma = sigma

    def convert(self, data):
        return np.random.normal(data, self.__sigma)
