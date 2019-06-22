# -*- coding: utf-8 -*-
from copy import deepcopy

def create(config_dict):
    if config_dict['type'] == 'michaelis_menten':
        p = [0., 0.]
        param_keys = ['b0', 'b1']
        model = MichaelisMentenEquation
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
        if type(p) is not list or \
            type(p[0]) is not float or \
            len(p) != expected_dof:
                raise Exception(
                        'Order of parameter({}) is invalid'.format(len(p)))
        # パラメータ
        self.p = p
        # モデル式
        self.f = lambda x, p: np.inf
        # モデル式のx0におけるテイラー展開(2次まで)
        self.tf1 = lambda x, x0, p: np.inf
        self.tf2 = lambda x, x0, p: np.inf
        # 残差式
        self.r = lambda x, p: np.inf
        # 残差式のx0におけるテイラー展開(2次まで)
        self.rf1 = lambda x, x0, p: np.inf
        self.rf2 = lambda x, x0, p: np.inf
        # 残差勾配
        self.rg = lambda x, p: [np.inf]

    def fx(self, x):
        return self.f(x, self.p)

    def update(self, p):
        if len(p) != len(self.p):
            print('Invalid dof({}). We ignore this updation.'.format(len(p)))
            return
        self.p = p

    def get_param(self):
        return deepcopy(self.p)

    def residual(self, x):
        return self.r(x, self.p)

    def gradient(self, x):
        return self.rg(x, self.p)

    def taylor(self, x, x0, order):
        if order == 1:
            taylor_func = self.tf1
        elif order == 2:
            taylor_func = self.tf2
        else:
            raise Exception('Invalid order of taylor function({}).'
                            'We ignore this process.'.format(order))
        return taylor_func(x, x0, self.p)

    def residual_taylor(self, x, x0, order):
        if order == 1:
            taylor_func = self.rf1
        elif order == 2:
            taylor_func = self.rf2
        else:
            raise Exception('Invalid order of taylor function({}).'
                            'We ignore this process.'.format(order))
        return taylor_func(x, x0, self.p)


# https://ja.wikipedia.org/wiki/%E3%82%AC%E3%82%A6%E3%82%B9%E3%83%BB%E3%83%8B%E3%83%A5%E3%83%BC%E3%83%88%E3%83%B3%E6%B3%95#%E4%BE%8B
# x1 = p0*x0 / (p1 + x0)
class MichaelisMentenEquation(Model):
    def __init__(self, p):
        super(MichaelisMentenEquation, self).__init__(p, 2)
        self.f = lambda x, p: p[0] * x[0] / (p[1] + x[0])
        self.r = lambda x, p: x[1] - self.f(x, p)
        drdx0 = lambda x, p: -x[0] / (p[1] + x[0])
        drdx1 = lambda x, p: p[0] * x[0] / (p[1] + x[0])**2
        self.rg = lambda x, p: [drdx0(x, p), drdx1(x, p)]


class Line2d(Model):
    def __init__(self, p):
        super(Line2d, self).__init__(p, 2)
        self.f = lambda x, p: p[0]*x[0] + p[1]
        self.r = lambda x, p: x[1] - self.f(x, p)
        drda = lambda x, p: -x[0]
        drdb = lambda x, p: -1.0
        self.rg = lambda x, p: [drda(x, p), drdb(x, p)]


class Circle2d(Model):
    def __init__(self, p):
        super(Circle2d, self).__init__(p, 3)
        self.f = lambda x, p: (x[0] - p[0])**2 + (x[1] - p[1])**2
        self.r = lambda x, p: p[2]**2 - self.f(x, p)

        drdx = lambda x, p: 2. * (x[0] - p[0])
        drdy = lambda x, p: 2. * (x[1] - p[1])
        drdr = lambda x, p: 2. * p[2]
        self.rg = lambda x, p: [drdx(x, p), drdy(x, p), drdr(x, p)]


class Curve2d2Order(Model):
    def __init__(self, p):
        super(Curve2d2Order, self).__init__(p, 3)
        # y = ax**2 + bx + c
        self.f = lambda x, p: p[0]*x[0]**2 + p[1]*x[0] + p[2]
        self.r = lambda x, p: x[1] - self.f(x, p)

        drda = lambda x, p: -x[0]**2
        drdb = lambda x, p: -x[0]
        drdc = lambda x, p: -1.
        self.rg = lambda x, p: [drda(x, p), drdb(x, p), drdc(x, p)]

        self.tf1 = lambda x, x0, p: self.f(x0, p) + \
                (2.*p[0]*x0[0] + p[1])*(x[0] - x0[0])
        self.tf2 = lambda x, x0, p: self.tf1(x, x0, p) + \
                p[0]*(x[0] - x0[0])**2


class Curve2d3Order(Model):
    def __init__(self, p):
        super(Curve2d3Order, self).__init__(p, 4)
        # y = ax**3 + bx**2 + cx + d
        self.f = lambda x, p: p[0]*x[0]**3 + p[1]*x[0]**2 + p[2]*x[0] + p[3]
        self.r = lambda x, p: x[1] - self.f(x, p)

        drda = lambda x, p: -x[0]**3
        drdb = lambda x, p: -x[0]**2
        drdc = lambda x, p: -x[0]
        drdd = lambda x, p: -1.
        self.rg = lambda x, p: [drda(x, p), drdb(x, p), drdc(x, p), drdd(x, p)]

        self.tf1 = lambda x, x0, p: self.f(x0, p) + \
                (3.*p[0]*x0[0]**2 + 2.*p[1]*x0[0] + p[2])*(x[0] - x0[0])
        self.tf2 = lambda x, x0, p: self.tf1(x, x0, p) + \
                (6.*p[0]*x0[0] + 2.*p[1])*(x[0] - x0[0])**2.