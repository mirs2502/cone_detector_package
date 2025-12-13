#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs_py import point_cloud2
import numpy as np
import math

class LandmarkLocalizer(Node):
    def __init__(self):
        super().__init__('landmark_localizer')

        # --- 設定 ---
        self.declare_parameter('min_shared_landmarks', 1) # 推定に必要な最低コーン数

        # 状態保持
        self.initial_landmarks = [] # 起動時のコーン配置 (Map) [(x, y), ...]
        self.is_map_initialized = False

        # --- Subscriber ---
        # ConeFusionNode が出力する「確定コーン」を購読
        self.sub_cones = self.create_subscription(
            PointCloud2,
            '/cone_centers', 
            self.callback_cones,
            10
        )

        # --- Publisher ---
        # 計算したロボットの位置を EKF へ
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped,
            '/landmark_pose',
            10
        )
        
        self.get_logger().info("LandmarkLocalizer started. Waiting for confirmed cones...")

    def callback_cones(self, msg):
        # PointCloud2 から (x, y) 座標を取り出す
        current_cones = []
        # skip_nans=True で無効な点を除去
        for p in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            current_cones.append(np.array([p[0], p[1]]))

        if not current_cones:
            return

        # --- Phase 1: 初期マップ作成 ---
        if not self.is_map_initialized:
            self.save_initial_map(current_cones)
            return

        # --- Phase 2: 自己位置推定 ---
        self.estimate_pose(current_cones)

    def save_initial_map(self, current_cones):
        # 起動直後の配置を「正解（地図）」として保存
        # ロボットの初期位置を (0, 0, 0) と定義する
        self.initial_landmarks = current_cones
        self.is_map_initialized = True
        self.get_logger().info(f"Map Initialized with {len(current_cones)} landmarks.")

    def estimate_pose(self, current_cones):
        """
        ICP(Iterative Closest Point)のようなマッチングを行い、位置ズレを計算する
        """
        # マッチングしたペアのズレ量を格納するリスト
        translations = [] 
        
        # 今回見えている各コーンについて、マップ上の最も近いコーンを探す
        for curr in current_cones:
            min_dist = float('inf')
            closest_map_lm = None
            
            for map_lm in self.initial_landmarks:
                dist = np.linalg.norm(curr - map_lm)
                if dist < min_dist:
                    min_dist = dist
                    closest_map_lm = map_lm
            
            # 距離が離れすぎている場合（例: 1.0m以上）は誤検知として無視
            if min_dist < 1.0:
                # ズレ = マップの位置(正解) - 現在の見え方
                # ロボットが前に進む(currが近づく)と、map - curr はプラスになる = ロボット位置
                translations.append(closest_map_lm - curr)

        if len(translations) < self.get_parameter('min_shared_landmarks').value:
            return

        # --- 位置 (X, Y) の計算 ---
        # 複数のペアがある場合は平均を取って精度を上げる
        translations = np.array(translations)
        avg_translation = np.mean(translations, axis=0)
        
        robot_x = float(avg_translation[0])
        robot_y = float(avg_translation[1])

        # --- 向き (Yaw) の計算 (コーンが2個以上見える場合のみ) ---
        # 今回はシンプルにするため、Yaw補正はEKFに任せて Position (X,Y) だけ更新する設定にします。
        # もし回転ズレも補正したい場合は、SVDによる回転行列の計算が必要です。
        
        self.publish_pose(robot_x, robot_y)

    def publish_pose(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map' # 地図座標系

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        
        # 回転は「不明(単位行列)」として出力し、共分散を大きくして無視させる手もあるが、
        # ここでは回転なし(0)として送る
        msg.pose.pose.orientation.w = 1.0

        # --- 共分散行列 (Covariance) ---
        # 対角成分: [x, y, z, roll, pitch, yaw]
        # 値が小さいほど「自信がある」という意味
        
        # X, Y は自信あり (0.05程度)
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        
        # Z, Roll, Pitch, Yaw は自信なし (大きな値を入れてEKFに無視させる)
        msg.pose.covariance[14] = 9999.0 # Z
        msg.pose.covariance[21] = 9999.0 # Roll
        msg.pose.covariance[28] = 9999.0 # Pitch
        msg.pose.covariance[35] = 9999.0 # Yaw (今回は計算していないため)

        self.pub_pose.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
