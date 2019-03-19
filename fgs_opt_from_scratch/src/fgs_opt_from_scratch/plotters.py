# -*- coding: utf-8 -*-
from copy import deepcopy
import matplotlib.pylab as plt
import numpy as np


def create(config_dict, model, data):
    if config_dict['type'] == 'residual2d':
        x_range = [min(data.T[0]), max(data.T[0])]
        y_range = [min(data.T[1]), max(data.T[1])]
        grid_num = config_dict['grid_num']
        return Residual2DPlotter(model, x_range, y_range, grid_num)
    elif config_dict['type'] == 'param2d':
        param_range = config_dict['param_range']
        grid_num = config_dict['grid_num']
        return Param2DPlotter(model, data, param_range, grid_num)
    else:
        raise NotImplementedError(
            'type {] is not implemented'.format(config_dict['type']))


class ClickableTaylorPlotter:
    u"""
    描画エリアをクリックしたポイントに応じたテイラー近似を表示するクラス
    """
    def __init__(self, function, x_range):
        self.__func = function
        self.__x = np.arange(x_range[0], x_range[1], 0.1)
        self.__y = [self.__func.fx(x) for x in self.__x]

        self.__fig = plt.figure('taylor')
        self.__fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.__ax = self.__fig.add_subplot(1, 1, 1)
        self.__ax.set_ylim(min(self.__y), max(self.__y))

    def show(self):
        self.__ax.plot(self.__x, self.__y, marker='.')
        self.__fig.show()

    def onclick(self, event):
        if event.xdata is None:
            print(event.xdata)
            return

        self.__fig.clf()
        self.__ax.cla()

        self.__fig = plt.figure('taylor')
        self.__ax = self.__fig.add_subplot(1, 1, 1)

        x = event.xdata

        taylor_x_array = []
        taylor1_y_array = []
        taylor2_y_array = []
        for new_x in self.__x:
            taylor_x_array.append(new_x)
            taylor1_y_array.append(
                    self.__func.taylor([x, self.__func.fx(x)], new_x, d=1))
            taylor2_y_array.append(
                    self.__func.taylor([x, self.__func.fx(x)], new_x, d=2))

        self.__ax.plot(self.__x, self.__y, marker='.')
        self.__ax.plot(taylor_x_array, taylor1_y_array, marker='.')
        self.__ax.plot(taylor_x_array, taylor2_y_array, marker='.')
        self.__ax.set_ylim(min(self.__y), max(self.__y))

        plt.pause(0.01)


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

        self.__ax.contour(self.__X, self.__Y, Z, cmap='hsv')
        im = self.__ax.pcolormesh(self.__X, self.__Y, Z, cmap='hsv')
        self.__fig.colorbar(im)

        plt.pause(0.01)


class Param2DPlotter:
    u"""
    現在のモデルパラメータを散りばめて
    残差分布を表示するクラス
    """
    def __init__(self, model, data, param_range, grid_num):
        u"""
        Args:
            model: 最適化対象モデル(models)
            data: 観測データ(np.array)
            param_range: パラメータのmin/max(list of float)
            grid_num: 分布分割数(int)
        """
        self.__model = deepcopy(model)
        param = self.__model.get_param()
        if len(param) != 2:
            raise Exception('Param2DPlotter support only 2D parameter')
        self.__data = data

        x = np.arange(
                param[0] - param_range[0], param[0] + param_range[0],
                (2. * param_range[0]) / grid_num)
        y = np.arange(
                param[1] - param_range[1], param[1] + param_range[1],
                (2. * param_range[1]) / grid_num)
        if len(x) > len(y):
            x = np.delete(x, -1)
        elif len(y) > len(x):
            y = np.delete(y, -1)
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
                self.__model.update([self.__X[i][j], self.__Y[i][j]])
                ess = 0
                for k in range(len(self.__data)):
                    ess += self.__model.residual(self.__data[k])**2
                Z_ele.append(ess)
            Z.append(Z_ele)
        Z = np.array(Z)

        self.__ax.contour(self.__X, self.__Y, Z, cmap='hsv')
        im = self.__ax.pcolormesh(self.__X, self.__Y, Z, cmap='hsv')
        self.__fig.colorbar(im)

        plt.pause(0.01)
