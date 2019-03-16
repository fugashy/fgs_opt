# -*- coding: utf-8 -*-
import numpy as np
import numpy.linalg as LA


def create(config_dict):
    if config_dict['type'] == 'gauss_newton':
        return gauss_newton
    else:
        raise NotImplementedError(
                '{} is not implemented'.format(config_dict['type']))

def gauss_newton(model, data):
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
        J[i] = model.gradient(data[i])

    # 近似されたヘッセ行列の一般逆行列(J.TとJの内積,その一般逆行列)
    AH = np.dot(J.T, J)
    AHInv = LA.pinv(AH)

    # 残差ベクトル
    R = np.array([model.residual(
        data[i]) for i in range(len(data))]).T

    # 更新ベクトル
    delta = np.dot(np.dot(AHInv, J.T), R).tolist()

    return delta
