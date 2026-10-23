# LiDARと機械学習を用いた移動ロボットの自律走行制御

東京大学 Physical AI講座 2026 応用編1 最終課題

## 1. 概要

ROS 2とGazeboを用いて、LiDARと機械学習による自律走行ロボットを開発しました。

参考記事で紹介されている手動操作ロボットを基に、KNN（k近傍法）による行動選択と状態遷移制御を組み合わせ、障害物の回避と目標地点への自律移動を実現しました。

## 2. 開発環境

- Windows 11 / WSL 2 / Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress
- Python 3
- NumPy / Matplotlib

## 3. 主な機能

- LiDARによる障害物検出
- KNNによる行動選択
- GOAL・TURN・PASSの状態遷移制御
- オドメトリを用いた目標地点への誘導
- 走行軌跡のCSV記録とグラフ化

## 4. 実行方法

ROS 2とGazeboの必要な環境を構築した後、以下を実行します。

```bash
cd ~/pai_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pai_nav demo.launch.py mode:=ai
```

## 5. 実験結果

2つの円柱障害物を配置したシミュレーション環境で、自律走行とゴール到達を確認しました。

- 推定走行距離：4.21 m
- シミュレーション時間：27.85秒
- ゴール到達：成功

※記録した1回の試行結果です。

## 6. 参考資料

N622「gazeboとROS2を連携してロボットを操作する」（Qiita）

https://qiita.com/N622/items/9d89f77d85d9da0af29e

## 7. デモ動画

YouTube：https://youtu.be/x6rbMA90Pso
