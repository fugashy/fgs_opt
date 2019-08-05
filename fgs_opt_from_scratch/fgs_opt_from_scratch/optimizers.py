# -*- coding: utf-8 -*-
from copy import deepcopy
import numpy as np
import numpy.linalg as LA

def create(model, data, updater, config_dict):
    tolerance = config_dict['tolerance']
    return Optimizer(model, data, updater, tolerance)


class Optimizer():
    def __init__(self, model, data, updater, tolerance):
        u"""
        コンストラクタ
        Args:
            model: データが従っているとされるモデル(models)
            data: 観測データ(numpy.array)
            update_func: パラメータを更新する関数(update_functions)
            tolerance: 最適化が完了したと判断するしきい値(float)
        Returns:
            なし
        """
        self.__model = model
        self.__data = data
        self.__updater = updater
        self.__tolerance = tolerance
        self.__num_iteration = 0

    def ess(self):
        u"""
        残差平方和(Error of sum squares)を計算する
        """
        ess = 0
        for i in range(len(self.__data)):
            ess += self.__model.residual(self.__data[i])**2

        return ess

    def likelihood(self):
        likelihood = 0.0
        for data in self.__data:
            likelihood += self.__model.likelihood(data)

        return likelihood

    def optimize(self, once=False):
        u"""
        更新ベクトルのノルムが一定値になるまでパラメータの更新を行う

        Args:
            once: 一度で止める(bool)

        Returns:
            更新回数(int)
        """
        try:
            while True:
                delta = self.__updater.update(
                        deepcopy(self.__model), deepcopy(self.__data))

                # 更新量が十分小さくなったら終了
                delta_norm = LA.norm(delta, ord=2)
                if delta_norm < self.__tolerance:
                    break

                # 更新
                param = np.array(self.__model.get_param())
                param -= delta
                self.__model.update(param)

                self.__num_iteration += 1
                if once:
                    break
        except KeyboardInterrupt:
            print('user interruption has occured')
        finally:
            return self.__num_iteration
