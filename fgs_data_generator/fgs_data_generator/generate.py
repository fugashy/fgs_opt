#!/usr/local/bin/python3
# -*- coding: utf-8 -*-
from sys import exit, argv

import numpy as np
import yaml



def generate(config_path, as_node=False):
    if as_node:
        from fgs_data_generator import data_2d, plot
    else:
        import data_2d, plot

    f = open(config_path, 'r')
    config_dict = yaml.load(f, Loader=yaml.FullLoader)

    data = data_2d.create(config_dict['model'])
    data.save('/tmp/generated_data.yaml')

    if 'plot' in config_dict and config_dict['plot']:
        plot.model_and_observed(data.gt, data.obs, ('x', 'f(x)'))

    print('Data is done !')
    return data

if __name__ == '__main__':
    if len(argv) != 2:
        print('usage: python3 generate PATH_TO_CONFIG.yaml')
        exit(1)

    generate(argv[1])

    exit(0)
