# -*- coding: utf-8 -*-

import numpy as np
import numpy.linalg as LA

def create(config_dict):
    if config_dict['type'] == 'gauss_newton':
        threshold = config_dict['threshold']
        return GaussNewton(threshold)
    else:
        raise NotImplementedError(
                '{} is not implemented'.format(config_dict['type']))

class GaussNewton:
    def __init__(self, threshold):
        self.__threshold = threshold
        self.__data = None
        self.__target = None

    def register(self, target, data):
        self.__target = target
        self.__data = data

    def residual(self):
        if self.__data is None or self.__target is None:
            print('Data or Model are empty')
            return 0

        # 残差平方和
        residual = 0;
        for i in range(len(self.__data)):
            residual += self.__target.residual(self.__data[i])**2

        return residual

    def optimize(self):
        if self.__data is None or self.__target is None:
            print('Data or Model are empty')
            return 0

        # 最適化
        num_iteration = 0
        while True:
            # ヤコビアン(パラメータ数 x パラメータ自由度)
            J = np.zeros((len(self.__data), len(self.__target.get_param())))
            for i in range(len(self.__data)):
                J[i] = self.__target.jacobian(self.__data[i])

            # 近似されたヘッセ行列の一般逆行列
            AH = np.dot(J.T, J)
            AHInv = LA.pinv(AH)

            # 残差ベクトル
            R = np.array([self.__target.residual(
                self.__data[i]) for i in range(len(self.__data))]).T

            # 更新ベクトル
            delta = np.dot(np.dot(AHInv, J.T), R).tolist()

            # 更新量が十分小さくなったら終了
            delta_norm = LA.norm(delta, ord=2)
            if delta_norm < self.__threshold:
                break

            # 更新
            param = np.array(self.__target.get_param())
            param -= delta
            self.__target.update(param)

            num_iteration += 1

        return num_iteration
