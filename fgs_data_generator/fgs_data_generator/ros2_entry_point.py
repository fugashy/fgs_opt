# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from fgs_data_generator.generate import generate as factory

def generate(args=None):
    rclpy.init(args=args)

    # Node name should be declared
    node = Node('generate')

    # Even if you pass parameters via command line by using yaml file,
    # should declare parameter name
    node.declare_parameter(name='config_path', value='')
    config_path = node.get_parameter('config_path').value
    print(config_path)

    try:
        factory(config_path)
    except Exception as e:
        # In python3 Exception has no attribute 'message'
        print(e)
        exit(-1)

    exit(0)
