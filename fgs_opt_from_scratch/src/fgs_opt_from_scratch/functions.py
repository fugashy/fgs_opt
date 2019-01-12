# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod
import numpy as np


class Func:
    __metaclass__ = ABCMeta

    def fx(self, x):
        raise NotImplementedError(
                'This is pure virtual class. '
                'Use inherited class.')

    def fxy(self):
        raise NotImplementedError(
                'This is pure virtual class. '
                'Use inherited class.')

    def normal_vector(self, v):
        raise NotImplementedError(
                'This is pure virtual class. '
                'Use inherited class.')

    def tangent(self, v, x):
        raise NotImplementedError(
                'This is pure virtual class. '
                'Use inherited class.')

    def taylor(self):
        raise NotImplementedError(
                'This is pure virtual class. '
                'Use inherited class.')


class Curve2dSampleFunc(Func):
    u"""
    fx = ax^2 + bx + c
    fxy = y - (ax^2 + bx + c)
        df/dx = -2ax - b
        df/dy = 1
    """
    def __init__(self, a, b, c):
        self.__fx = lambda x: a*pow(x, 2.) + b*x + c
        self.__fxy = lambda v: v[1] - (a*pow(v[0], 2.) + b*v[0] + c)
        self.__nv = \
            lambda v: [-2.*a*v[0] - b, 1]
        self.__tngt = \
            lambda v, x: v[1] - (self.__nv(v)[0]*(x - v[0]) / self.__nv(v)[1])
        self.__taylor1 = \
            lambda v, x: self.__fx(v[0]) + 2.*a*v[0]*(x - v[0])
        self.__taylor2 = \
            lambda v, x: self.__taylor1(v, x) + a * pow(x - v[0], 2.)

    def fx(self, x):
        return self.__fx(x)

    def fxy(self):
        return self.__fxy

    def normal_vector(self, v):
        return self.__nv(v)

    def tangent(self, v, x):
        return self.__tngt(v, x)

    def taylor(self, v, x, d=1):
        if d == 1:
            return self.__taylor1(v, x)
        else:
            return self.__taylor2(v, x)
