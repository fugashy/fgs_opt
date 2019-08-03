# -*- coding: utf-8 -*-
from sys import exit

import cv2
import rclpy
from rclpy.node import Node
import yaml

from fgs_opt_from_scratch import (
    models, optimizers, updaters, plotters
)
from fgs_data_generator.generator import generate_2d


def optimize(args=None):
    rclpy.init(args=args)

    node = Node('optimize')

    node.declare_parameter(name='opt_config_path', value='')
    node.declare_parameter(name='data_config_path', value='')

    # データ
    data_config_path = node.get_parameter('data_config_path').value
    data = generate_2d(data_config_path)

    opt_config_path = node.get_parameter('opt_config_path').value
    with open(opt_config_path, 'r') as f:
        opt_config_dict = yaml.load(f, Loader=yaml.FullLoader)

    # モデル
    model = models.create(opt_config_dict['model'])

    # パラメータ更新器
    updater = updaters.create(opt_config_dict['updater'])

    # 最適化処理クラス
    optimizer = optimizers.create(
        model, data, updater, opt_config_dict['optimizer'])

    # 描画クラス(optional)
    plotter = None
    if 'plotter' in opt_config_dict and opt_config_dict['plotter']:
        plotter = plotters.create(opt_config_dict['plotter'], model, data)

    # 1周期毎に停止(optional)
    once = False
    if 'once' in opt_config_dict and opt_config_dict['once']:
        once = opt_config_dict['once']

    print('initial param               : {}'.format(model.get_param()))
    print('initial error of sum squares: {}'.format(optimizer.ess()))

    print('Optimize...')

    previous_num = 0
    num_iteration = 0
    while True:
        if plotter:
            print('Plotting...')
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
        print('current param               : {}'.format(model.get_param()))
        print('current error of sum squares: {}'.format(optimizer.ess()))

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
