# -*- coding: utf-8 -*-
import numpy as np


def create(conf_dict):
    if conf_dict['type'] == 'curve2d':
        start = conf_dict['start']
        end = conf_dict['end']
        step = conf_dict['step']
        a = conf_dict['a']
        b = conf_dict['b']
        c = conf_dict['c']
        # 3次関数or2次関数
        if 'd' in conf_dict and conf_dict['d']:
            d = conf_dict['d']
            p = [a, b, c, d]
            return Curve2d(start, end, step, p)
        else:
            p = [a, b, c]
            return Curve2d(start, end, step, p)
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
    elif conf_dict['type'] == 'michaelis_menten':
        b1 = conf_dict['b1']
        b2 = conf_dict['b2']
        start = conf_dict['start']
        end = conf_dict['end']
        step = conf_dict['step']
        return MichaelisMenten(b1, b2, start, end, step)
    else:
        raise NotImplementedError(
                '{} is not implemented.'.format(conf_dict['type']))


class Curve2d():
    u"""
    """
    def __init__(self, start, end, step, p):
        if len(p) == 3:
            self.__y = lambda x, p: p[0] * x**2 + p[1] * x + p[2]
        elif len(p) == 4:
            self.__y = lambda x, p: p[0] * x**3 + p[1] * x**2 + p[2] * x + p[3]
        else:
            raise Exception('Invalid parameter length.')

        if start >= end or end < step or step <= 0:
            raise Exception('Invalid x configulation.')

        self.__start = start
        self.__end = end
        self.__step = step
        self.__p = p

    def create(self):
        return [[x, self.__y(x, self.__p)]
                for x in np.arange(self.__start, self.__end, self.__step)]


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


class MichaelisMenten():
    def __init__(self, b1, b2, start, end, step):
        self.__b1 = b1
        self.__b2 = b2
        self.__s = start
        self.__e = end
        self.__st = step

    def create(self):
        x_range = np.arange(self.__s, self.__e, self.__st)

        f = lambda x: self.__b1 * x / (self.__b2 + x)

        return [ [x, f(x)] for x in x_range]
