# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod
import numpy as np


def create(config_dict):
    if config_dict['type'] == 'Curve2Order':
        a = config_dict['a']
        b = config_dict['b']
        c = config_dict['c']
        return Curve2Order(a, b, c)
    elif config_dict['type'] == 'Curve3Order':
        a = config_dict['a']
        b = config_dict['b']
        c = config_dict['c']
        d = config_dict['d']
        return Curve3Order(a, b, c, d)
    else:
        raise NotImplementedError(
            'type {] is not implemented'.format(config_dict['type']))

class Func:
    __metaclass__ = ABCMeta

    def fx(self, x):
        raise NotImplementedError(
                'This is pure virtual class. '
                'Use inherited class.')

    def taylor(self, v, x, d=1):
        raise NotImplementedError(
                'This is pure virtual class. '
                'Use inherited class.')


class Curve2Order(Func):
    u"""
    fx = ax^2 + bx + c
    fxy = y - (ax^2 + bx + c)
        df/dx = -2ax - b
        df/dy = 1
    """
    def __init__(self, a, b, c):
        self.__fx = lambda x: a*pow(x, 2.) + b*x + c
        self.__taylor1 = \
            lambda v, x: self.__fx(v[0]) + (2.*a*v[0] + b)*(x - v[0])
        self.__taylor2 = \
            lambda v, x: self.__taylor1(v, x) + a * pow(x - v[0], 2.)

    def fx(self, x):
        return self.__fx(x)

    def taylor(self, v, x, d=1):
        if d == 1:
            return self.__taylor1(v, x)
        else:
            return self.__taylor2(v, x)


class Curve3Order(Func):
    def __init__(self, a, b, c, d):
        u"""
        y = ax^3 + bx^2 + cx + d
        """
        self.__fx = lambda x: a*pow(x, 3.) + b*pow(x, 2.) + c*x + d
        self.__taylor1 = \
            lambda v, x: self.__fx(v[0]) + \
                         ((3.*a*pow(v[0], 2.)) + (2.*b*v[0]) + c)*(x - v[0])
        self.__taylor2 = \
            lambda v, x: self.__taylor1(v, x) + (6.*a*v[0] + 2.*b)*pow(x - v[0], 2.)

    def fx(self, x):
        return self.__fx(x)

    def taylor(self, v, x, d=1):
        if d == 1:
            return self.__taylor1(v, x)
        else:
            return self.__taylor2(v, x)
