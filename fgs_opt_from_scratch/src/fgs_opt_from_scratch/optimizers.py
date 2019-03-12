# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod
import numpy as np
import numpy.linalg as LA

def create(target, data, config_dict):
    if config_dict['type'] == 'gauss_newton':
        threshold = config_dict['threshold']
        return GaussNewton(target, data, threshold)
    else:
        raise NotImplementedError(
                '{} is not implemented'.format(config_dict['type']))

class Optimizer():
    __metaclass__ = ABCMeta

    def __init__(self, target, data):
        self.data = data
        self.target = target

    def residual(self):
        if self.data is None or self.target is None:
            print('Data or Model are empty')
            return 0

        # 残差平方和
        residual = 0;
        for i in range(len(self.data)):
            residual += self.target.residual(self.data[i])**2

        return residual

    @abstractmethod
    def optimize(self):
        raise NotImplementedError()


class GaussNewton(Optimizer):
    def __init__(self, target, data, threshold):
        super(GaussNewton, self).__init__(target, data)
        self.__threshold = threshold

    def optimize(self):
        if self.data is None or self.target is None:
            print('Data or Model are empty')
            return 0

        # 最適化
        num_iteration = 0
        try:
            while True:
                # ヤコビアン(パラメータ数 x パラメータ自由度)
                J = np.zeros((len(self.data), len(self.target.get_param())))
                for i in range(len(self.data)):
                    J[i] = self.target.gradient(self.data[i])

                # 近似されたヘッセ行列の一般逆行列
                AH = np.dot(J.T, J)
                AHInv = LA.pinv(AH)

                # 残差ベクトル
                R = np.array([self.target.residual(
                    self.data[i]) for i in range(len(self.data))]).T

                # 更新ベクトル
                delta = np.dot(np.dot(AHInv, J.T), R).tolist()

                # 更新量が十分小さくなったら終了
                delta_norm = LA.norm(delta, ord=2)
                if delta_norm < self.__threshold:
                    break

                # 更新
                param = np.array(self.target.get_param())
                param -= delta
                self.target.update(param)

                num_iteration += 1
        except KeyboardInterrupt:
            print('user interruption has occured')
        finally:
            return num_iteration
