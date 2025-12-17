#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs_py import point_cloud2
import numpy as np
import math

# TF2 for frame transformation
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped


class LandmarkLocalizer(Node):
    def __init__(self):
        super().__init__('landmark_localizer')

        # --- パラメータ設定 ---
        self.declare_parameter('min_shared_landmarks', 2)  # Yaw計算には最低2点必要
        self.declare_parameter('matching_threshold', 1.0)  # マッチング距離閾値[m]
        self.declare_parameter('target_frame', 'base_link')  # 処理用フレーム（TF変換）
        self.declare_parameter('output_frame', 'odom')  # 出力ポーズのフレーム

        # 状態保持
        self.initial_landmarks = []  # 起動時のコーン配置 (Map) [(x, y), ...]
        self.is_map_initialized = False

        # --- TF2 Buffer (座標変換用) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriber ---
        self.sub_cones = self.create_subscription(
            PointCloud2,
            '/confirmed_cones',
            self.callback_cones,
            10
        )

        # --- Publisher ---
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped,
            '/landmark_pose',
            10
        )

        self.get_logger().info("LandmarkLocalizer started with Yaw estimation enabled.")

    def callback_cones(self, msg):
        """コーン点群を受信した際のコールバック"""
        target_frame = self.get_parameter('target_frame').value
        source_frame = msg.header.frame_id

        # frame_id が空の場合はデフォルトを使用
        if not source_frame:
            source_frame = 'lidar'  # または 'odom'
            self.get_logger().warn_once(f"PointCloud2 has empty frame_id, assuming '{source_frame}'")

        # PointCloud2 から座標を抽出
        current_cones = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            # TF変換が必要な場合は変換する
            if source_frame != target_frame:
                try:
                    # PointStamped を作成
                    point_in = PointStamped()
                    point_in.header = msg.header
                    point_in.header.frame_id = source_frame
                    point_in.point.x = float(p[0])
                    point_in.point.y = float(p[1])
                    point_in.point.z = 0.0

                    # 変換実行 (最新のTFを使用)
                    point_out = self.tf_buffer.transform(
                        point_in, target_frame, timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    current_cones.append(np.array([point_out.point.x, point_out.point.y]))
                except TransformException as ex:
                    self.get_logger().warning(
                        f"TF transform failed: {ex}"
                    )
                    return
            else:
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
        """起動時のコーン配置を「正解マップ」として保存"""
        self.initial_landmarks = current_cones
        self.is_map_initialized = True
        self.get_logger().info(f"Map initialized with {len(current_cones)} landmarks.")

    def estimate_pose(self, current_cones):
        """
        SVD を用いて回転(Yaw)と並進(X, Y)を同時に推定する
        """
        matching_threshold = self.get_parameter('matching_threshold').value
        min_landmarks = self.get_parameter('min_shared_landmarks').value

        # マッチングしたペアを格納: [(current, map), ...]
        matched_pairs = []

        for curr in current_cones:
            min_dist = float('inf')
            closest_map_lm = None

            for map_lm in self.initial_landmarks:
                dist = np.linalg.norm(curr - map_lm)
                if dist < min_dist:
                    min_dist = dist
                    closest_map_lm = map_lm

            if min_dist < matching_threshold:
                matched_pairs.append((curr, closest_map_lm))

        if len(matched_pairs) < min_landmarks:
            self.get_logger().info(
                f"Not enough matched landmarks: {len(matched_pairs)} < {min_landmarks} (initial: {len(self.initial_landmarks)}, current: {len(current_cones)})"
            )
            return

        # --- SVD による回転・並進の推定 ---
        src = np.array([p[0] for p in matched_pairs])  # 現在の観測
        dst = np.array([p[1] for p in matched_pairs])  # マップ上の位置

        # 重心を計算
        centroid_src = np.mean(src, axis=0)
        centroid_dst = np.mean(dst, axis=0)

        # 中心化
        src_centered = src - centroid_src
        dst_centered = dst - centroid_dst

        # SVD で回転行列を計算
        H = src_centered.T @ dst_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # 反射行列の補正 (det(R) < 0 の場合)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # Yaw 角を計算 (2D回転行列から抽出)
        yaw = math.atan2(R[1, 0], R[0, 0])

        # 並進を計算: t = dst_centroid - R @ src_centroid
        t = centroid_dst - R @ centroid_src

        robot_x = float(t[0])
        robot_y = float(t[1])

        self.get_logger().info(
            f"Pose estimated: X={robot_x:.3f}, Y={robot_y:.3f}, Yaw={math.degrees(yaw):.1f}°"
        )

        self.publish_pose(robot_x, robot_y, yaw)

    def publish_pose(self, x, y, yaw):
        """推定した位置と向きをパブリッシュ"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('output_frame').value

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        # Yaw から Quaternion に変換
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # --- 共分散行列 ---
        # X, Y, Yaw は自信あり
        msg.pose.covariance[0] = 0.05   # X
        msg.pose.covariance[7] = 0.05   # Y
        msg.pose.covariance[35] = 0.1   # Yaw (回転は少し不確実性を持たせる)

        # Z, Roll, Pitch は自信なし
        msg.pose.covariance[14] = 9999.0  # Z
        msg.pose.covariance[21] = 9999.0  # Roll
        msg.pose.covariance[28] = 9999.0  # Pitch

        self.pub_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
