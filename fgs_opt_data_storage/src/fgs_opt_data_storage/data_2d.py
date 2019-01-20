# -*- coding: utf-8 -*-
import numpy as np


def create(conf_dict):
    if conf_dict['type'] == 'curve':
        start = conf_dict['start']
        end = conf_dict['end']
        step = conf_dict['step']
        alpha = conf_dict['alpha']
        beta = conf_dict['beta']
        return Curve(start, end, step, alpha, beta)
    elif conf_dict['type'] == 'ellipse':
        a = conf_dict['a']
        b = conf_dict['b']
        rotation = conf_dict['rotation']
        translation = conf_dict['translation']
        theta_start = conf_dict['theta_start']
        theta_end = conf_dict['theta_end']
        theta_step = conf_dict['theta_step']
        return Ellipse(
                a, b, rotation, translation,
                theta_start, theta_end, theta_step)
    elif conf_dict['type'] == 'const':
        translation = conf_dict['translation']
        num = conf_dict['num']
        return Const(translation, num)
    elif conf_dict['type'] == 'line':
        a = conf_dict['a']
        b = conf_dict['b']
        start = conf_dict['start']
        end = conf_dict['end']
        step = conf_dict['step']
        return Line(a, b, start, end, step)
    else:
        raise NotImplementedError(
                '{} is not implemented.'.format(conf_dict['type']))


def curve(start, end, step, alpha, beta):
    u"""
    Create curve data
    """
    if start >= end or end < step:
        raise Exception('Invalid x configulation.')

    # y = a*x^2 + b
    y = lambda x, alpha, beta: alpha * pow(x, 2) + beta

    return [[x, y(x, alpha, beta)] for x in np.arange(start, end, step)]


class Curve():
    u"""
    """
    def __init__(self, start, end, step, alpha, beta):
        if start >= end or end < step or step <= 0:
            raise Exception('Invalid x configulation.')

        self.__start = start
        self.__end = end
        self.__step = step
        self.__alpha = alpha
        self.__beta = beta

    def create(self):
        return curve(self.__start,
                     self.__end,
                     self.__step,
                     self.__alpha,
                     self.__beta)


class Ellipse():
    def __init__(
            self, a, b, rotation, translation,
            theta_start, theta_end, theta_step):
        self.__a = a
        self.__b = b
        self.__rot = rotation
        self.__trs = translation
        self.__ts = theta_start
        self.__te = theta_end
        self.__tstep = theta_step

    def create(self):
        theta_range = np.arange(self.__ts, self.__te, self.__tstep)

        trs = self.__trs

        xy = lambda t: \
            np.array([self.__a * np.cos(t) + trs[0],
                      self.__b * np.sin(t) + trs[1]])
        rot_mat = np.array([[np.cos(self.__rot), -np.sin(self.__rot)],
                            [np.sin(self.__rot), np.cos(self.__rot)]])

        return [np.dot(rot_mat, xy(t)) for t in theta_range]


class Const():
    def __init__(self, translation, num):
        self.__trs = translation
        self.__num = num

    def create(self):
        return [self.__trs for i in range(self.__num)]


class Line():
    def __init__(self, a, b, start, end, step):
        self.__a = a
        self.__b = b
        self.__s = start
        self.__e = end
        self.__st = step

    def create(self):
        x_range = np.arange(self.__s, self.__e, self.__st)

        f = lambda x: self.__a * x + self.__b

        return [ [x, f(x)] for x in x_range]
