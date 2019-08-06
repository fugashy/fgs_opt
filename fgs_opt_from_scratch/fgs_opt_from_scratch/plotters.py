# -*- coding: utf-8 -*-
from copy import deepcopy
import matplotlib.pylab as plt
import numpy as np


def create(config_dict, model, data):
    npdata = np.array(data)
    if config_dict['type'] == 'residual2d':
        x_range = [min(npdata.T[0]), max(npdata.T[0])]
        y_range = [min(npdata.T[1]), max(npdata.T[1])]
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
    def __init__(self, model, x_list, base_x=0.0):
        self._model = model
        self._x_list = x_list
        self._y_list = [self._model.fx([x]) for x in self._x_list]

        self._fig = plt.figure('taylor')
        self._fig.canvas.mpl_connect('button_press_event', self._onclick)
        self._ax = self._fig.add_subplot(1, 1, 1)

        self._taylor_y_array = []

        self.base_x = base_x
        self.update()

    def plot(self):
        self._fig.clf()
        self._ax.cla()

        self._fig = plt.figure('taylor')
        self._ax = self._fig.add_subplot(1, 1, 1)

        self._ax.plot(self._x_list, self._y_list, linestyle='solid', label='origin')
        for order in range(len(self._taylor_y_array)):
            label = '{} order approximation'.format(order + 1)
            self._ax.plot(
                self._x_list, self._taylor_y_array[order],
                linestyle='dashed', label=label)
        self._ax.plot(
            [self.base_x], self._model.fx([self.base_x]),
            marker='.', label='approximation base point')

        self._ax.set_title('Taylor expansion')
        self._ax.legend(loc='lower right')
        self._ax.set_ylim(min(self._y_list), max(self._y_list))

        plt.pause(0.01)

    def _onclick(self, event):
        if event.xdata is None:
            return
        self.base_x = event.xdata
        self.update()
        self.plot()

    def update(self):
        u"""現在の基点でのテイラー近似結果をx_range範囲内で計算しなおす

        modelからは[y1, y2]という形で返ってくる
        plotするためには，[[y1_0, y1_1, ..., y1_n], [y2_0, y2_1, ..., y2_n]]という形が望ましい
        その形にするための処理もする
        """
        self._taylor_y_array = [[] for i in range(self._model.taylor_num())]

        for x in self._x_list:
            y_list = self._model.taylor([x], [self.base_x])
            for i in range(len(y_list)):
                self._taylor_y_array[i].append(y_list[i])


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
        # xとyで要素数が異なったら端を削って調整
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
        self.__model = model
        self.__data = data
        self._param_range = param_range
        self._grid_num = grid_num

        # 更新毎にたどってきたパラメータの軌跡がみたい
        self._param_history = [[], []]

        self.__X = None
        self.__Y = None
        self._reset_range()
        self.__first = True

    def plot(self):
        if not self.__first:
            self.__fig.clf()
            self.__ax.cla()
        self.__first = False

        self.__fig = plt.figure('residual')
        self.__ax = self.__fig.add_subplot(1, 1, 1)

        # 現在のパラメータを中心にプロットするためのrangeを計算
        self._reset_range()

        # 現在のパラメータを履歴に追加
        current_param = self.__model.get_param()
        self._param_history[0].append(current_param[0])
        self._param_history[1].append(current_param[1])

        # パラメータを書き換えるのでコピーして参照している他のモジュールへの影響を防ぐ
        model = deepcopy(self.__model)
        Z = self._compute_residual_distribution(model)

        im = self.__ax.pcolormesh(self.__X, self.__Y, Z, cmap='hsv')
        self.__ax.plot(self._param_history[0], self._param_history[1], marker='.')
        self.__fig.colorbar(im)

        plt.pause(0.01)

    def _reset_range(self):
        param = self.__model.get_param()
        if len(param) != 2:
            raise Exception('Param2DPlotter support only 2D parameter')
        param_range = self._param_range
        grid_num = self._grid_num
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

    def _compute_residual_distribution(self, model):
        Z = []
        for i in range(len(self.__X)):
            Z_ele = []
            for j in range(len(self.__Y)):
                # 書き換え
                model.update([self.__X[i][j], self.__Y[i][j]])
                ess = 0
                for k in range(len(self.__data)):
                    # 誤差があまりにも大きくなることがあるので対数にする
                    # 大小関係は変わらない
                    ess += np.log(model.residual(self.__data[k])**2)
                Z_ele.append(ess)
            Z.append(Z_ele)
        Z = np.array(Z)

        return Z
