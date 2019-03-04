# -*- coding: utf-8 -*-
import data_2d
import noise_model
import saver
import plot

import numpy as np
import yaml


def generate_2d(config_path):
    f = open(config_path, 'r')
    config_dict = yaml.load(f)

    m = data_2d.create(config_dict['model'])
    nm = noise_model.create(config_dict['noise_model'])
    fs = saver.create(config_dict['saver'])

    # generate
    data = m.create()
    noisy_data = [nm.convert(xy) for xy in data]
    fs.save(noisy_data)
    if config_dict['plot']:
        plot.model_and_observed(data, noisy_data, ('x', 'f(x)'))

    print('Data is done !')
