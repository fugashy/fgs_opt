# -*- coding: utf-8 -*-
from sys import exit

import cv2
import rclpy
from rclpy.node import Node
import yaml

from fgs_opt_from_scratch import (
    models, optimizers, updaters, plotters
)


def optimize(args=None):
    rclpy.init(args=args)

    node = Node('optimize')

    node.declare_parameter(name='config_path', value='')
    node.declare_parameter(name='data_path', value='')
    config_path = node.get_parameter('config_path').value
    data_path = node.get_parameter('data_path').value

    with open(config_path, 'r') as f:
        config_dict = yaml.load(f, Loader=yaml.FullLoader)

    # データ
    cv_yaml = cv2.FileStorage(data_path, cv2.FileStorage_READ)
    data = cv_yaml.getNode('data').mat()

    # モデル
    model = models.create(config_dict['model'])

    # パラメータ更新器
    updater = updaters.create(config_dict['updater'])

    # 最適化処理クラス
    optimizer = optimizers.create(model, data, updater, config_dict['optimizer'])

    # 描画クラス(optional)
    plotter = None
    if 'plotter' in config_dict and config_dict['plotter']:
        plotter = plotters.create(config_dict['plotter'], model, data)

    # 1周期毎に停止(optional)
    once = False
    if 'once' in config_dict and config_dict['once']:
        once = config_dict['once']

    print('initial param               : {}'.format(model.get_param()))
    print('initial error of sum squares: {}'.format(optimizer.ess()))

    print('optimize...')

    previous_num = 0
    num_iteration = 0
    while True:
        if plotter:
            plotter.plot()
        if once:
            try:
                print('press enter to next iteration')
                input()
            except:
                pass
        num_iteration = optimizer.optimize(once=once)
        if previous_num == num_iteration:
            break
        previous_num = num_iteration


    print('num of iteration          : {}'.format(num_iteration))
    print('final param               : {}'.format(model.get_param()))
    print('final error of sum squares: {}'.format(optimizer.ess()))
    try:
        input('\npress enter to terminate')
    except:
        pass

    exit(0)


def view_taylor(args=None):
    rclpy.init(args=args)
    node = Node('view_taylor')

    node.declare_parameter(name='config_path', value='')
    config_path = node.get_parameter('config_path').value

    f = open(config_path, 'r')
    config_dict = yaml.load(f, Loader=yaml.FullLoader)

    model = models.create(config_dict['model'])
    x_range = config_dict['x_range']
    plotter = plotters.ClickableTaylorPlotter(model, x_range)
    plotter.plot()

    try:
        input('press enter to terminate')
    except:
        pass
