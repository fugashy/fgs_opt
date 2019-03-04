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


def as_cv(file_path, all_data):
    file_handle = cv2.FileStorage(file_path, cv2.FileStorage_WRITE)
    file_handle.write('data', np.array(all_data))


def save_ba_in_large_as_cv(file_path, all_data):
    file_handle = cv2.FileStorage(file_path, cv2.FileStorage_WRITE)
    file_handle.write('observations', np.array(all_data[0]))
    file_handle.write('camera_parameters', np.array(all_data[1]))
    file_handle.write('points', np.array(all_data[2]))


class SaveAsCV():
    def __init__(self, output_path):
        self.__output_path = output_path

    def save(self, data):
        as_cv(self.__output_path, data)
