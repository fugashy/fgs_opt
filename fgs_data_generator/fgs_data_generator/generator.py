# -*- coding: utf-8 -*-
from sys import exit, argv

import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from fgs_data_generator import (
    data_2d, plot
)

def generate_2d(config_path):
    f = open(config_path, 'r')
    config_dict = yaml.load(f, Loader=yaml.FullLoader)

    data = data_2d.create(config_dict['model'])
    data.save('/tmp/data_2d.yaml')

    if 'plot' in config_dict and config_dict['plot']:
        plot.model_and_observed(data.gt, data.obs, ('x', 'f(x)'))

    print('Data is done !')
    return data

def ros2_entry_point(args=None):
    rclpy.init(args=args)

    # Node name should be declared
    node = Node('generate_data_2d')

    # Even if you pass parameters via command line by using yaml file,
    # should declare parameter name
    node.declare_parameter(name='config_path', value='')
    config_path = node.get_parameter('config_path').value

    try:
        generate_2d(config_path)
    except Exception as e:
        # In python3 Exception has no attribute 'message'
        print(e)
        exit(-1)

    exit(0)
