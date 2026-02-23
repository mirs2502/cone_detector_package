# cone_detector - カラーコーン検出パッケージ

LiDARとカメラのセンサーフュージョンを用いてカラーコーンを検出し、コーン群が囲むエリア（凸包）を特定するROS 2パッケージです。

## 概要

**役割:** センサーフュージョンによる「カラーコーン検出」と「エリア（凸包）の特定」

このパッケージは以下の処理を行います：
1. **LiDARによる形状検出**: 点群からクラスタリングでコーン候補を抽出
2. **カメラによる色検出**: HSV色空間でカラー領域を検出
3. **センサーフュージョン**: 形状と色の両方が一致したものを「本物のコーン」として確認
4. **エリア特定**: 確認されたコーン群を囲む凸包（Polygon）を計算

## 主要な機能

### 距離フィルタリング
- **3m以内の点のみを検出**: 遠距離のノイズを除去し、検出精度を向上
- `scan_to_pointcloud`ノードで実装（PCLフィルタリング）

### コーン検出パイプライン
1. `/scan` (LaserScan) → `scan_to_pointcloud` → `/point_cloud` (PointCloud2)
2. `/point_cloud` → `cone_cluster_node` → `/cone_clusters` (MarkerArray)
3. カメラ画像 → `cone_color_detector_node` → `/color_regions` (ColorRegions)
4. `cone_fusion_node` → `/confirmed_cones` (MarkerArray)
5. `cone_area_node` → `/cone_area` (PolygonStamped)

### Nav2統合
- **cone_costmap_layer**: 検出されたコーンをNav2のコストマップに統合するプラグイン
- ナビゲーション時にコーンを障害物として認識

## 依存関係のインストール

### ROS 2パッケージ
```bash
sudo apt update
sudo apt install ros-humble-pcl-conversions ros-humble-cv-bridge \
                 ros-humble-image-transport ros-humble-image-geometry \
                 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
                 ros-humble-v4l2-camera ros-humble-camera-calibration \
                 ros-humble-rqt-reconfigure ros-humble-rqt-image-view \
                 ros-humble-nav-msgs ros-humble-geometry-msgs \
                 ros-humble-nav2-costmap-2d
```

### ライブラリ
```bash
sudo apt install libpcl-dev libopencv-dev
```

## ビルド

```bash
cd ~/mirs_ws
colcon build --packages-select cone_detector --symlink-install
source install/setup.bash
```

## 起動方法

### 単体起動
```bash
ros2 launch cone_detector cone_detection.launch.py
```

### フルシステム起動（推奨）
```bash
# LIDAR、カメラ、ナビゲーション、コーン検出をすべて起動
ros2 launch mirs system_bringup.launch.py
```

## トピック

### サブスクライブ
- `/scan` (sensor_msgs/LaserScan) - LiDARスキャンデータ
- `/image_raw` (sensor_msgs/Image) - カメラ画像

### パブリッシュ
- `/point_cloud` (sensor_msgs/PointCloud2) - LiDAR点群（3m以内フィルタ済み）
- `/cone_clusters` (visualization_msgs/MarkerArray) - LiDARで検出したコーン候補
- `/color_regions` (custom msg) - カメラで検出した色領域
- `/confirmed_cones` (visualization_msgs/MarkerArray) - センサーフュージョンで確認されたコーン
- `/cone_area` (geometry_msgs/PolygonStamped) - コーン群の凸包

## パラメータ

主要な設定は各ノードのソースコードで定義されています：

### scan_to_pointcloud
- `max_distance`: 3.0m - 検出距離の上限

### cone_cluster_node
- クラスタリング距離閾値
- 最小/最大クラスタサイズ

### cone_color_detector_node
- HSV色閾値（赤、青、緑など）

## ノード一覧

| ノード名 | 実行ファイル | 説明 |
|---------|-------------|------|
| scan_to_pointcloud | `scan_to_pointcloud` | LaserScan → PointCloud2変換 + 距離フィルタ |
| cone_cluster_node | `cone_cluster_node` | 点群クラスタリング |
| cone_color_detector_node | `cone_color_detector_node` | カメラ色検出 |
| cone_fusion_node | `cone_fusion_node` | センサーフュージョン |
| cone_area_node | `cone_area_node` | 凸包計算 |

## RViz可視化

RViz2で以下のトピックを追加：
- `/cone_clusters` (MarkerArray) - LiDAR検出（形状のみ）
- `/confirmed_cones` (MarkerArray) - フュージョン確認済み（赤で表示）
- `/cone_area` (Polygon) - コーン群の凸包エリア

## トラブルシューティング

### コーンが検出されない
- LiDARの範囲を確認（3m以内に配置）
- カメラのキャリブレーション確認
- 照明条件を確認（色検出はHSVベース）

### 誤検出が多い
- HSV閾値を調整
- クラスタリング距離閾値を調整
- 3m距離フィルタが機能しているか確認

### Nav2で障害物として認識されない
- `cone_costmap_layer`プラグインが読み込まれているか確認
- `nav2_params.yaml`でプラグインが有効になっているか確認

## 関連パッケージ

- **coverage_planner**: `/cone_area`から経路生成（矩形パターン）
- **bt_pkg**: ビヘイビアツリーでミッション実行
- **mirs**: ロボット本体制御とナビゲーション

## ライセンス

このパッケージのライセンスについては、LICENSEファイルを参照してください。
