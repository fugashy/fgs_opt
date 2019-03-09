# fgs_opt_from_scratch

最適化を一から実装する試み

参考文献: これなら分かる最適化数学

<https://www.kyoritsu-pub.co.jp/kenpon/bookDetail/9784320017863>

# 実行可能ファイル

- newton_sample

  wikipediaのガウス・ニュートン法の例に上がっているミカエリス・メンテン式のパラメータ推定を

  実装したもの

  ```bash
  rosrun fgs_opt_from_scratch gauss_newton_sample
  ```

- view_taylor

  曲線の任意の点における1次,2次近似をプロットする
  任意の点は、グラフをクリックしたときのx軸に依る

  ```bash
  rosrun fgs_opt_from_scratch view_taylor _config_path:=/path/to/this/package/config/taylor.yaml
  ```

  更新し過ぎるとバグる現象あり
