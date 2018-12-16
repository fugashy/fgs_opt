# -*- coding: utf-8 -*-
import numpy as np


def create(conf_dict):
    if conf_dict['type'] == 'curve':
        start = conf_dict['start']
        end = conf_dict['end']
        step = conf_dict['step']
        alpha = conf_dict['alpha']
        return Curve(start, end, step, alpha)
    else:
        raise NotImplementedError(
                '{} is not implemented.'.format(conf_dict['type']))


def curve(start, end, step, alpha):
    u"""
    Create curve data
    """
    if start >= end or end < step:
        raise Exception('Invalid x configulation.')

    # y = a*x^2
    y = lambda x, alpha: alpha * pow(x, 2)

    return [[x, y(x, alpha)] for x in np.arange(start, end, step)]


class Curve():
    u"""
    """
    def __init__(self, start, end, step, alpha):
        if start >= end or end < step or step <= 0:
            raise Exception('Invalid x configulation.')

        self.__start = start
        self.__end = end
        self.__step = step
        self.__alpha = alpha

    def create(self):
        return curve(self.__start, self.__end, self.__step, self.__alpha)
