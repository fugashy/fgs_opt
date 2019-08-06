# -*- coding: utf-8 -*-
from abc import ABC, abstractmethod
import time

import numpy as np

from fgs_opt_from_scratch import models, plotters

def create(config_dict, model):
    if config_dict['type'] == 'clickable':
        return ClickableTaylorExpansionContext(
            config_dict, model)
    elif config_dict['type'] == 'scan_range':
        return ScanningRangeTaylorExpansionContext(
            config_dict, model)
    else:
        raise NotImplementedError(
            '{} is not implemented'.format(config_dict['context']))


class TaylorExpansionContext(ABC):
    def __init__(self, model):
        self._model = model

    @abstractmethod
    def run_expansion(self):
        raise NotImplementedError('To developer, inherit this class')


class ClickableTaylorExpansionContext(TaylorExpansionContext):
    def __init__(self, config_dict, model):
        super().__init__(model)

        x_range = config_dict['x_range']
        x_step_ratio = config_dict['x_step_ratio']
        x_step = (x_range[1] - x_range[0]) * x_step_ratio
        self._x_list = np.arange(x_range[0], x_range[1], x_step)

        self._plotter = plotters.ClickableTaylorPlotter(
            self._model, self._x_list, self._x_list[0])

    def run_expansion(self):
        self._plotter.plot()


class ScanningRangeTaylorExpansionContext(TaylorExpansionContext):
    def __init__(self, config_dict, model):
        super().__init__(model)

        x_range = config_dict['x_range']
        x_step_ratio = config_dict['x_step_ratio']
        x_step = (x_range[1] - x_range[0]) * x_step_ratio
        self._x_list = np.arange(x_range[0], x_range[1], x_step)

        self._cycle_sec = config_dict['cycle_sec']

        self._plotter = plotters.ClickableTaylorPlotter(
            self._model, self._x_list, self._x_list[0])

    def run_expansion(self):
        self._plotter.plot()
        input('press enter to start')
        for x in self._x_list:
            time.sleep(self._cycle_sec)
            self._plotter.base_x = x
            self._plotter.update()
            self._plotter.plot()
