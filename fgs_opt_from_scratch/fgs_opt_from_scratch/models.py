# -*- coding: utf-8 -*-
from copy import deepcopy
from math import cos, sin, sqrt

def create(config_dict):
    if config_dict['type'] == 'const2d':
        p = [0., 0.]
        param_keys = ['x', 'y']
        model = Const
    elif config_dict['type'] == 'line2d':
        p = [0., 0.]
        param_keys = ['a', 'b']
        model = Line2d
    elif config_dict['type'] == 'circle2d':
        p = [0., 0., 0.]
        param_keys = ['x', 'y', 'r']
        model = Circle2d
    elif config_dict['type'] == 'curve2d_2order':
        p = [0., 0., 0.]
        param_keys = ['a', 'b', 'c']
        model = Curve2d2Order
    elif config_dict['type'] == 'curve2d_3order':
        p = [0., 0., 0., 0.]
        param_keys = ['a', 'b', 'c', 'd']
        model = Curve2d3Order
    elif config_dict['type'] == 'michaelis_menten':
        p = [0., 0.]
        param_keys = ['b0', 'b1']
        model = MichaelisMentenEquation
    elif config_dict['type'] == 'cos':
        p = [0., 0.]
        param_keys = ['a', 'b']
        model = Cos
    else:
        raise NotImplementedError(
            'type {] is not implemented'.format(config_dict['type']))

    for i, param_key in enumerate(param_keys):
        if param_key in config_dict and config_dict[param_key]:
            p[i] = config_dict[param_key]

    return model(p)


class Model(object):
    def __init__(self, p, expected_dof):
        u"""
        Args:
            p: パラメータ(list of float)
            expected_dof: 期待するdof(int)
        """
        if type(p) is not list:
            raise Exception('Parameter should be formed as list')

        if type(p[0]) is not float:
            raise Exception('Parameter type should be float')

        if len(p) != expected_dof:
            raise Exception(
                    'Order of parameter({}) is invalid'.format(len(p)))

        # パラメータ
        self._p = p
        # モデル式
        self._f = lambda x, p: np.inf
        # 残差式
        self._r = lambda x, p: np.inf
        # 残差勾配
        self._rg = lambda x, p: [np.inf]
        # モデル式のx0におけるテイラー展開(とりあえず2次まで)
        self._tf = [lambda x, x0, p: np.inf for i in range(2)]

    def fx(self, x):
        return self._f(x, self._p)

    def update(self, p):
        if len(p) != len(self._p):
            print('Invalid dof({}). We ignore this updation.'.format(len(p)))
            return
        self._p = p

    def get_param(self):
        return deepcopy(self._p)

    def residual(self, x):
        return self._r(x, self._p)

    def gradient(self, x):
        return self._rg(x, self._p)

    def taylor(self, x, x0):
        return \
            [
                self._tf[order](x, x0, self._p)
                for order in range(len(self._tf))
            ]

    def taylor_num(self):
        return len(self._tf)


class Const(Model):
    def __init__(self, p):
        super(Const, self).__init__(p, 2)
        self._f = lambda x, p: sqrt(x[0]**2 + x[1]**2)
        self._r = lambda x, p: (x[0] - p[0])**2 + (x[1] - p[1])**2

        drdx = lambda x, p: -2.0 * (x[0] - p[0])
        drdy = lambda x, p: -2.0 * (x[1] - p[1])
        self._rg = lambda x, p: [drdx(x, p), drdy(x, p)]


# https://ja.wikipedia.org/wiki/%E3%82%AC%E3%82%A6%E3%82%B9%E3%83%BB%E3%83%8B%E3%83%A5%E3%83%BC%E3%83%88%E3%83%B3%E6%B3%95#%E4%BE%8B
# x1 = p0*x0 / (p1 + x0)
class MichaelisMentenEquation(Model):
    def __init__(self, p):
        super(MichaelisMentenEquation, self).__init__(p, 2)
        self._f = lambda x, p: p[0] * x[0] / (p[1] + x[0])
        self._r = lambda x, p: x[1] - self._f(x, p)
        drdx0 = lambda x, p: -x[0] / (p[1] + x[0])
        drdx1 = lambda x, p: p[0] * x[0] / (p[1] + x[0])**2
        self._rg = lambda x, p: [drdx0(x, p), drdx1(x, p)]


class Line2d(Model):
    def __init__(self, p):
        super(Line2d, self).__init__(p, 2)
        self._f = lambda x, p: p[0]*x[0] + p[1]
        self._r = lambda x, p: x[1] - self._f(x, p)
        drda = lambda x, p: -x[0]
        drdb = lambda x, p: -1.0
        self._rg = lambda x, p: [drda(x, p), drdb(x, p)]


class Circle2d(Model):
    def __init__(self, p):
        super(Circle2d, self).__init__(p, 3)
        self._f = lambda x, p: (x[0] - p[0])**2 + (x[1] - p[1])**2
        self._r = lambda x, p: p[2]**2 - self._f(x, p)

        drdx = lambda x, p: 2. * (x[0] - p[0])
        drdy = lambda x, p: 2. * (x[1] - p[1])
        drdr = lambda x, p: 2. * p[2]
        self._rg = lambda x, p: [drdx(x, p), drdy(x, p), drdr(x, p)]


class Curve2d2Order(Model):
    def __init__(self, p):
        super(Curve2d2Order, self).__init__(p, 3)
        # y = ax**2 + bx + c
        self._f = lambda x, p: p[0]*x[0]**2 + p[1]*x[0] + p[2]
        self._r = lambda x, p: x[1] - self._f(x, p)

        drda = lambda x, p: -x[0]**2
        drdb = lambda x, p: -x[0]
        drdc = lambda x, p: -1.
        self._rg = lambda x, p: [drda(x, p), drdb(x, p), drdc(x, p)]

        # f(x0) + {1/1! * f'(x0) * (x - x0)}
        self._tf[0] = lambda x, x0, p: self._f(x0, p) + \
                (2.*p[0]*x0[0] + p[1])*(x[0] - x0[0])
        # f(x0) + {1/1! * f'(x0) * (x - x0)} + {1/2! * f''(x0) * (x - x0)^2
        self._tf[1] = lambda x, x0, p: self._tf[0](x, x0, p) + \
                p[0]*(x[0] - x0[0])**2


class Curve2d3Order(Model):
    def __init__(self, p):
        super(Curve2d3Order, self).__init__(p, 4)
        # y = ax**3 + bx**2 + cx + d
        self._f = lambda x, p: p[0]*x[0]**3 + p[1]*x[0]**2 + p[2]*x[0] + p[3]
        self._r = lambda x, p: x[1] - self._f(x, p)

        drda = lambda x, p: -x[0]**3
        drdb = lambda x, p: -x[0]**2
        drdc = lambda x, p: -x[0]
        drdd = lambda x, p: -1.
        self._rg = lambda x, p: [drda(x, p), drdb(x, p), drdc(x, p), drdd(x, p)]

        self._tf[0] = lambda x, x0, p: self._f(x0, p) + \
                (3.*p[0]*x0[0]**2 + 2.*p[1]*x0[0] + p[2])*(x[0] - x0[0])
        self._tf[1] = lambda x, x0, p: self._tf[0](x, x0, p) + \
                (6.*p[0]*x0[0] + 2.*p[1])*(x[0] - x0[0])**2.


# Optimization is not good
# Maybe, residual function is not good
class Cos(Model):
    def __init__(self, p):
        # p = [a, b]
        super(Cos, self).__init__(p, 2)
        # y = a*cos(bx)
        self._f = lambda x, p: p[0] * cos(p[1] * x[0])
        self._r = lambda x, p: x[1] - self._f(x, p)

        drda = lambda x, p: -cos(p[1] * x[0])
        drdb = lambda x, p: x[0] * p[0] * sin(p[1] * x[0])
        self._rg = lambda x, p: [drda(x, p), drdb(x, p)]

        self._tf[0] = lambda x, x0, p: self._f(x0, p) - \
            p[0] * p[1] * sin(p[1] * x0[0]) * (x[0] - x0[0])
        self._tf[1] = lambda x, x0, p: self._tf[0](x, x0, p) - \
            0.5 * p[0] * p[1]**2 * cos(p[1] * p[1]) * (x[0] - x0[0])**2
