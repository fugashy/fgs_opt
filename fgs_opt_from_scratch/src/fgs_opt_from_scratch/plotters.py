# -*- coding: utf-8 -*-
import matplotlib.pylab as plt
import numpy as np


def create(config_dict, model, data):
    if config_dict['type'] == 'residual2d':
        x_range = [min(data.T[0]), max(data.T[0])]
        y_range = [min(data.T[1]), max(data.T[1])]
        grid_num = config_dict['grid_num']
        return Residual2DPlotter(model, x_range, y_range, grid_num)
    else:
        raise NotImplementedError(
            'type {] is not implemented'.format(config_dict['type']))


class Residual2DPlotter:
    u"""
    現在のモデルパラメータにデータを散りばめて
    残差分布を表示するクラス
    """
    def __init__(self, model, x_range, y_range, grid_num):
        u"""
        Args:
            model: 最適化対象モデル(models)
            x_range: 観測データのx座標min/max(float)
            y_range: 観測データのy座標min/max(float)
            grid_num: 分布分割数(int)
        """
        self.__model = model
        x = np.arange(x_range[0], x_range[1], (x_range[1] - x_range[0]) / grid_num)
        y = np.arange(y_range[0], y_range[1], (y_range[1] - y_range[0]) / grid_num)
        self.__X, self.__Y = np.meshgrid(x, y)
        self.__first = True

    def plot(self):
        if not self.__first:
            self.__fig.clf()
            self.__ax.cla()
        self.__first = False

        self.__fig = plt.figure('residual')
        self.__ax = self.__fig.add_subplot(1, 1, 1)

        Z = []
        for i in range(len(self.__X)):
            Z_ele = []
            for j in range(len(self.__Y)):
                Z_ele.append(
                        self.__model.residual([self.__X[i][j], self.__Y[i][j]]))
            Z.append(Z_ele)
        Z = np.array(Z)

        im = self.__ax.pcolormesh(self.__X, self.__Y, Z, cmap='hsv')
#       im = self.__ax.contour(X, Y, np.array(Z), cmap='hsv')
        self.__fig.colorbar(im)

        plt.pause(0.01)
