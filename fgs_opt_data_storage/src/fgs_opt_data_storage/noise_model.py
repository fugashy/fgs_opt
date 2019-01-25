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
        if type(sigma) is not list:
            raise Exception('Invalid type of sigma, should be list')

        is_valid = [ele > -0. for ele in sigma]
        if False in is_valid:
            raise Exception('sigma of gaussian should be greater than 0')

        self.__sigma = np.array(sigma)

    def convert(self, data):
        return np.random.normal(data, self.__sigma)
