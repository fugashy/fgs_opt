# -*- coding: utf-8 -*-

def create(config_dict):
    if config_dict['type'] == 'MichaelisMentenEquation':
        return MichaelisMentenEquation()
    else:
        raise NotImplementedError(
            'type {] is not implemented'.format(config_dict['type']))

# https://ja.wikipedia.org/wiki/%E3%82%AC%E3%82%A6%E3%82%B9%E3%83%BB%E3%83%8B%E3%83%A5%E3%83%BC%E3%83%88%E3%83%B3%E6%B3%95#%E4%BE%8B
class MichaelisMentenEquation:
    def __init__(self):
        self.__b = [0., 0.]
        # v(s) = b0s/(b2 + s)
        self.__v = lambda s, b: b[0]*s / (b[1] + s)
        # 残差
        self.__r = lambda x, b: x[1] - self.__v(x[0], b)
        # 残差のヤコビアン要素
        self.__dfdb0 = lambda s, b: -s / (b[1] + s)
        self.__dfdb1 = lambda s, b: b[0]*s / (b[1] + s)**2

    def update(self, b):
        u"""
        パラメータを更新する

        Args:
            b: ミカエリス・メンテン式のパラメータ(list)

        Returns:
            なし
        """
        if len(b) != self.dof():
            print('Invalid dof. We ignore this updation.')
            return

        self.__b = b

    def dof(self):
        u"""
        パラメータの自由度

        Args:
            なし

        Returns:
            2
        """
        return len(self.__b)

    def residual(self, x):
        u"""
        入力データに対する残差を計算する

        Args:
            データ(list)

        Returns:
            残差(float)
        """
        return self.__r(x, self.__b)

    def jacobian(self, x):
        u"""
        入力データに対するヤコビ行列の要素を返す

        Args:
            データ(list)

        Retuns:
            ヤコビ要素(list)
        """
        return [self.__dfdb0(x[0], self.__b), self.__dfdb1(x[0], self.__b)]


