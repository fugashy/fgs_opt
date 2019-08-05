# -*- coding: utf-8 -*-
import cv2
import numpy as np


def create(conf_dict):
    if conf_dict['type'] == 'curve2d':
        return Curve2d.create(conf_dict)
    elif conf_dict['type'] == 'ellipse':
        return Ellipse.create(conf_dict)
    elif conf_dict['type'] == 'const':
        return Const.create(conf_dict)
    elif conf_dict['type'] == 'line':
        return Line.create(conf_dict)
    elif conf_dict['type'] == 'michaelis_menten':
        return MichaelisMenten.create(conf_dict)
    elif conf_dict['type'] == 'cos':
        return Cos.create(conf_dict)
    else:
        raise NotImplementedError(
                '{} is not implemented.'.format(conf_dict['type']))


class Data2d():
    def __init__(self, std_dev):
        self._noiser = lambda gt: np.random.normal(
            np.array(gt), np.array(std_dev))

        self._gt = None
        self._obs = None
        self._cov = list(np.array(std_dev)**2)
        self._center = None

    @property
    def gt(self):
        return self._gt

    @property
    def obs(self):
        return self._obs

    @property
    def cov(self):
        return self._cov

    @property
    def center(self):
        return self._center

    def save(self, filepath):
        file_handle = cv2.FileStorage(filepath, cv2.FileStorage_WRITE)
        file_handle.write('ground_truth', np.array(self._gt))
        file_handle.write('observation', np.array(self._obs))
        file_handle.write('center', np.array(self._center))
        file_handle.write('covariance', np.array(self._cov))

    def _generate(self, generate_data):
        self._gt = generate_data()
        self._obs = self._noiser(self._gt)
        self._center = list(np.mean(self._obs, axis=0, keepdims=True))
        self._obs = list(self._obs)

class Curve2d(Data2d):
    u"""
    """
    @staticmethod
    def create(conf_dict):
        start = conf_dict['start']
        end = conf_dict['end']
        step = conf_dict['step']
        a = conf_dict['a']
        b = conf_dict['b']
        c = conf_dict['c']
        std_dev = conf_dict['std_dev']
        # 3次関数or2次関数
        if 'd' in conf_dict and conf_dict['d']:
            d = conf_dict['d']
            p = [a, b, c, d]
            return Curve2d(start, end, step, p, std_dev)
        else:
            p = [a, b, c]
            return Curve2d(start, end, step, p, std_dev)

    def __init__(self, s, e, st, p, std_dev):
        super(Curve2d, self).__init__(std_dev)
        if len(p) == 3:
            f = lambda x: p[0] * x**2 + p[1] * x + p[2]
        elif len(p) == 4:
            f = lambda x: p[0] * x**3 + p[1] * x**2 + p[2] * x + p[3]
        else:
            raise Exception('Invalid parameter length.')

        generate_data = lambda : [[x, f(x)] for x in np.arange(s, e, st)]
        self._generate(generate_data)


class Ellipse(Data2d):
    @staticmethod
    def create(conf_dict):
        a = conf_dict['a']
        b = conf_dict['b']
        rotation = conf_dict['rotation']
        translation = conf_dict['translation']
        theta_start = conf_dict['theta_start']
        theta_end = conf_dict['theta_end']
        theta_step = conf_dict['theta_step']
        std_dev = conf_dict['std_dev']
        return Ellipse(
                a, b, rotation, translation,
                theta_start, theta_end, theta_step, std_dev)

    def __init__(self, a, b, r, trs, s, e, st, std_dev):
        super(Ellipse, self).__init__(std_dev)
        theta_range = np.arange(s, e, st)

        xy = lambda t: np.array(
            [a * np.cos(t) + trs[0], b * np.sin(t) + trs[1]])
        rot_mat = np.array(
            [
                [np.cos(r), -np.sin(r)],
                [np.sin(r), np.cos(r)]
            ])
        generate_data = lambda: [rot_mat @  xy(t) for t in theta_range]
        self._generate(generate_data)


class Const(Data2d):
    @staticmethod
    def create(conf_dict):
        translation = conf_dict['translation']
        num = conf_dict['num']
        std_dev = conf_dict['std_dev']
        return Const(translation, num, std_dev)

    def __init__(self, trs, num, std_dev):
        super(Const, self).__init__(std_dev)
        generate_data = lambda : [trs for i in range(num)]
        self._generate(generate_data)


class Line(Data2d):
    @staticmethod
    def create(conf_dict):
        a = conf_dict['a']
        b = conf_dict['b']
        start = conf_dict['start']
        end = conf_dict['end']
        step = conf_dict['step']
        std_dev = conf_dict['std_dev']
        return Line(a, b, start, end, step, std_dev)

    def __init__(self, a, b, start, end, step, std_dev):
        super(Line, self).__init__(std_dev)
        x_range = np.arange(start, end, step)
        f = lambda x: a * x + b
        generate_data = lambda: [[x, f(x)] for x in x_range]
        self._generate(generate_data)


class MichaelisMenten(Data2d):
    @staticmethod
    def create(conf_dict):
        b1 = conf_dict['b1']
        b2 = conf_dict['b2']
        start = conf_dict['start']
        end = conf_dict['end']
        step = conf_dict['step']
        std_dev = conf_dict['std_dev']
        return MichaelisMenten(b1, b2, start, end, step, std_dev)

    def __init__(self, b1, b2, s, e, st, std_dev):
        super(MichaelisMenten, self).__init__(std_dev)
        x_range = np.arange(s, e, st)
        f = lambda x: b1 * x / (b2 + x)
        generate_data = lambda: [[x, f(x)] for x in x_range]
        self._generate(generate_data)


class Cos(Data2d):
    @staticmethod
    def create(conf_dict):
        a = conf_dict['a']
        b = conf_dict['b']
        start = conf_dict['start']
        end = conf_dict['end']
        step = conf_dict['step']
        std_dev = conf_dict['std_dev']
        return Cos(a, b, start, end, step, std_dev)

    def __init__(self, a, b, start, end, step, std_dev):
        super(Cos, self).__init__(std_dev)
        x_range = np.arange(start, end, step)
        f = lambda x: a * np.cos(b * x)
        generate_data = lambda: [[x, f(x)] for x in x_range]
        self._generate(generate_data)
