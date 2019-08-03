# -*- coding: utf-8 -*-
import numpy as np
import numpy.linalg as LA


def create(config_dict):
    if config_dict['type'] == 'gauss_newton':
        return GaussNewton()
    elif config_dict['type'] == 'levenberg_marquardt':
        weight = config_dict['weight']
        return LevenbergMarquardt(weight)
    else:
        raise NotImplementedError(
                '{} is not implemented'.format(config_dict['type']))


class GaussNewton():
    u"""
    ガウスニュートン法を用いて、
    データからパラメータを最適に近づけるパラメータを算出するクラス
    """
    def __init__(self):
        pass

    def update(self, model, data):
        u"""
        ガウスニュートン法でパラメータを更新する
        Args:
            model: データが従っているとされるモデル(modelsモジュール)
            data: 観測データ(numpy.array)
        Returns:
            更新されたパラメータ(numpy.array)
        """
        # ヤコビアン(パラメータ数 x パラメータ自由度)
        J = np.zeros((len(data), len(model.get_param())))
        for i in range(len(data)):
            J[i] = model.residual_gradient(data[i])

        # 近似されたヘッセ行列の一般逆行列(J.TとJの内積,その一般逆行列)
        AH = np.dot(J.T, J)
        AHInv = LA.pinv(AH)

        # 残差ベクトル
        R = np.array([model.residual(
            data[i]) for i in range(len(data))]).T

        # 更新ベクトル
        delta = np.dot(np.dot(AHInv, J.T), R).tolist()

        return delta


class LevenbergMarquardt():
    u"""
    レーベンバーグマーカート法を用いて
    データからパラメータを最適に近づけるパラメータを算出するクラス
    """
    def __init__(self, weight):
        u"""
        Args:
            weight: 更新に勾配に掛け合わせる重み(1より大きい値）(float)
        """
        if weight <= 0. or 1. < weight:
            raise Exception('Weight of LM should be in range between 0 and 1')

        self.__weight = weight
        self.__previous_ess = None

    def update(self, model, data):
        u"""
        レーベンバーグマーカート法でパラメータを更新する
        Args:
            model: データが従っているとされるモデル(modelsモジュール)
            data: 観測データ(numpy.array)
        Returns:
            更新されたパラメータ(numpy.array)
        """
        # 初期の残差の平方和を求める
        if self.__previous_ess is None:
            self.__previous_ess = 0.
            for i in range(len(data)):
                self.__previous_ess += model.residual(data[i])**2

        # ヤコビアン(パラメータ数 x パラメータ自由度)
        J = np.zeros((len(data), len(model.get_param())))
        for i in range(len(data)):
            J[i] = model.residual_gradient(data[i])

        # 近似されたヘッセ行列(J.TとJの内積)
        AH = np.dot(J.T, J)

        # 近似されたヘッセ行列の対角成分のみを残した行列
        DAH = np.diag(AH.diagonal())

        while True:
            # 重み
            C = np.eye(DAH.shape[0]) * self.__weight

            # 残差ベクトル
            R = np.array([model.residual(
                data[i]) for i in range(len(data))]).T

            # 更新パラメータ
            delta = np.dot(np.dot(LA.pinv(AH + C * DAH), J.T), R)

            # 更新したパラメータで残差平方和を求める
            current_param = model.get_param()
            model.update(current_param - delta)
            current_ess = 0.
            for i in range(len(data)):
                current_ess += model.residual(data[i])**2

            if current_ess > self.__previous_ess:
                # 前回の値よりおおきくなるようなら重みを更新して再度計算する
                self.__weight *= 10.
                # パラメータは更新しない
                model.update(current_param)
            else:
                # そうでないなら更新する
                self.__weight /= 10.
                self.__previous_ess = current_ess
                break

        return delta
