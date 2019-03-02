# -*- coding: utf-8 -*-
import numpy as np
import cv2


def create(conf_dict):
    if conf_dict['type'] == 'cv':
        output_path = conf_dict['output_path']
        return SaveAsCV(output_path)
    else:
        raise NotImplementedError(
                '{} is not implemented.'.format(conf_dict['type']))


def as_cv(file_path, all_data, with_extend=False):
    file_handle = cv2.FileStorage(file_path, cv2.FileStorage_WRITE)
    if with_extend:
        file_handle.write('data', np.array(all_data[0]))
        index = 0
        for data in all_data[1:]:
            file_handle.write('extend_data{}'.format(index), np.array(data))
            index += 1
    else:
        file_handle.write('data', np.array(all_data))


class SaveAsCV():
    def __init__(self, output_path):
        self.__output_path = output_path

    def save(self, data):
        as_cv(self.__output_path, data, with_extend=False)
