# -*- coding: utf-8 -*-
import numpy as np
import numpy.linalg as LA

def create(model, data, update_func, config_dict):
    tolerance = config_dict['tolerance']
    return Optimizer(model, data, update_func, tolerance)

class Optimizer():
    def __init__(self, model, data, update_func, tolerance):
        u"""
        コンストラクタ
        Args:
            model: データが従っているとされるモデル(models)
            data: 観測データ(numpy.array)
            update_func: パラメータを更新する関数(update_functions)
        Returns:
            なし
        """
        self.__model = model
        self.__data = data
        self.__update_func = update_func
        self.__tolerance = tolerance
        self.__num_iteration = 0

    def ess(self):
        u"""
        残差平方和(Error of sum squares)を計算する
        """
        ess = 0;
        for i in range(len(self.__data)):
            ess += self.__model.residual(self.__data[i])**2

        return ess

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
                delta = self.__update_func(self.__model, self.__data)

                # 更新量が十分小さくなったら終了
                delta_norm = LA.norm(delta, ord=2)
                if delta_norm < self.__tolerance:
                    break
                elif once:
                    break

                # 更新
                param = np.array(self.__model.get_param())
                param -= delta
                self.__model.update(param)

                self.__num_iteration += 1
        except KeyboardInterrupt:
            print('user interruption has occured')
        finally:
            return self.__num_iteration
