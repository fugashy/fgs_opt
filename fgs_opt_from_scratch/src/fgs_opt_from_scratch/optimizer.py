# -*- coding: utf-8 -*-

import numpy as np
import numpy.linalg as LA

class Newton:
    def __init__(self, target, data):
        self.__target = target
        self.__data = data

    def residual(self):
        # 残差平方和
        residual = 0;
        for i in range(len(self.__data)):
            residual += self.__target.residual(self.__data[i])**2

        return residual

    def optimize(self, threshold=0.00001):
        # 最適化
        num_iteration = 0
        while True:
            # ヤコビアン(パラメータ数 x パラメータ自由度)
            J = np.zeros((len(self.__data), len(self.__target.get_param())))
            for i in range(len(self.__data)):
                J[i] = self.__target.jacobian(self.__data[i])
            JInv = LA.pinv(J)

            # 残差ベクトル
            R = np.array([self.__target.residual(
                self.__data[i]) for i in range(len(self.__data))]).T

            # 更新ベクトル
            delta = np.dot(JInv, R).tolist()

            # 更新量が十分小さくなったら終了
            delta_norm = LA.norm(delta, ord=2)
            if delta_norm < threshold:
                break

            # 更新
            param = np.array(self.__target.get_param())
            param -= delta
            self.__target.update(param)

            num_iteration += 1

        return num_iteration
