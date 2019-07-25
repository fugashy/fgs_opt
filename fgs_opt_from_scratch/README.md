# fgs_opt_from_scratch

最適化を一から実装する試み

参考文献: これなら分かる最適化数学

<https://www.kyoritsu-pub.co.jp/kenpon/bookDetail/9784320017863>

# 実行可能ファイル

- optimize

  設定ファイルにしたがってモデルパラメータの推定を行う

  ```bash
  ros2 run fgs_opt_from_scratch optimize __params:=PARAM_FILE.yaml
  ```

  PARAM_FILEのサンプル

  ```yaml
  optimize:
    ros__parameters:
      config_path: PATH_TO_THIS_PKG/config/optimize.yaml
      data_path: /tmp/data_XXXX.yaml
  ```

- view_taylor

  曲線の任意の点における1次,2次近似をプロットする
  任意の点は、グラフをクリックしたときのx軸に依る

  ```bash
  ros2 run fgs_opt_from_scratch view_taylor __params:=PARAM_FILE.yaml
  ```

  PARAM_FILEのサンプル

  ```yaml
  view_taylor:
    ros__parameters:
      config_path: PATH_TO_THIS_PKG/config/taylor.yaml
  ```
