# カラーコーン検出 & 経路生成パッケージ (cone_detector / coverage_planner)

LiDARとカメラのセンサーフュージョン を用いてカラーコーンを検出し、そのコーン群が囲むエリア（凸包） 内のジグザグ走行経路を生成するプロジェクトです。

## 概要

このプロジェクトは、以下の2つのROS 2パッケージで構成されています。

1.  **`cone_detector`**
    * **役割:** センサーフュージョンによる「カラーコーン検出」と「エリア（凸包）の特定」
    * LiDARの点群から「形」（クラスタ）を、カメラから「色」（HSV）を検出し、両方が一致したもの を「本物のコーン（`/confirmed_cones`）」とします。
    * 最後に、本物のコーン群を囲む `/cone_area` (Polygon) を発行します。

2.  **`coverage_planner`**
    * **役割:** 「経路計画（カバレッジパスプランニング）」
    * `/cone_area` (Polygon) をサブスクライブします。
    * 多角形の内部を走行するためのジグザグ経路（Boustrophedonパターン）を計算し、`/coverage_path` (Path) として発行します。
    * （センサーデバッグ用に、ダミーの四角形を発行する `dummy_area_publisher` も含みます）

---

## 依存関係のインストール

**必要なもの (apt):**
```bash
sudo apt update
# ROS 2 関連
sudo apt install ros-humble-pcl-conversions ros-humble-cv-bridge \
                 ros-humble-image-transport ros-humble-image-geometry \
                 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
                 ros-humble-v4l2-camera ros-humble-camera-calibration \
                 ros-humble-rqt-reconfigure ros-humble-rqt-image-view \
                 ros-humble-nav-msgs ros-humble-geometry-msgs

# ライブラリ本体
sudo apt install libpcl-dev libopencv-dev
```
─────────────────────────────────────────────────────────────────────────

## 起動方法
1. **`(ターミナル1) ロボット本体（Odom）とLiDARドライバを起動`**
   ```bash
   # (例: mirs パッケージの場合)
   ros2 launch mirs mirs.launch.py serial_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0
   # ポートは環境で違う場合があるので都度合わせるように
   ```
2. **`(ターミナル2) rvizを起動`**
   ```bash
   rviz2
   ```
3. **`(ターミナル3) ジグザグ経路生成ノードを起動
   ```bash
   ros2 run coverage_planner zigzag_generator
   ```
