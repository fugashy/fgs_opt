# -*- coding: utf-8 -*-
import data_2d
import noise_model
import saver
import plot

import yaml


def generate_2d(config_path):
    f = open(config_path, 'r')
    config_dict = yaml.load(f)

    m = data_2d.create(config_dict['model'])
    nm = noise_model.create(config_dict['noise_model'])
    fs = saver.create(config_dict['saver'])

    # generate
    data = m.create()
    noisy_data = [[x, nm.convert(y)] for x, y in data]
    fs.save(noisy_data)
    plot.model_and_observed(data, noisy_data, ('x', 'f(x)'))
