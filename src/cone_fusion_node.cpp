#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 3D->2D投影 ライブラリ
#include <image_geometry/pinhole_camera_model.h>

// TF2 (座標変換) ライブラリ
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// メッセージ同期 ライブラリ (今回は使わない シンプル実装)
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

// 連続検出追跡用の構造体
struct TrackedCandidate {
  pcl::PointXYZ position;      // 3D位置
  int detection_count;         // 連続検出回数
  rclcpp::Time last_detection; // 最後に検出された時刻
};

class ConeFusionNode : public rclcpp::Node {
public:
  ConeFusionNode()
      : Node("cone_fusion_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // フュージョン閾値パラメータ（ピクセル距離）
    this->declare_parameter("fusion_pixel_threshold", 100.0);
    this->get_parameter("fusion_pixel_threshold", fusion_pixel_threshold_);
    RCLCPP_INFO(this->get_logger(), "Fusion pixel threshold: %.1f",
                fusion_pixel_threshold_);

    // 連続検出カウンタのパラメータ
    this->declare_parameter("min_confirmation_count",
                            3); // 確定に必要な連続検出回数
    this->declare_parameter("tracking_distance",
                            0.3);                     // 同一候補とみなす距離[m]
    this->declare_parameter("tracking_timeout", 1.0); // 追跡タイムアウト[秒]
    this->get_parameter("min_confirmation_count", min_confirmation_count_);
    this->get_parameter("tracking_distance", tracking_distance_);
    this->get_parameter("tracking_timeout", tracking_timeout_);
    RCLCPP_INFO(this->get_logger(),
                "Confirmation count: %d, Tracking dist: %.2f m",
                min_confirmation_count_, tracking_distance_);

    // 距離整合性チェックのパラメータ
    this->declare_parameter("cone_real_size", 0.2); // コーンの実サイズ[m]（幅）
    this->declare_parameter("distance_tolerance",
                            0.5); // 距離許容誤差の割合（50%）
    this->get_parameter("cone_real_size", cone_real_size_);
    this->get_parameter("distance_tolerance", distance_tolerance_);
    RCLCPP_INFO(this->get_logger(),
                "Distance check: cone_size=%.2f m, tolerance=%.0f%%",
                cone_real_size_, distance_tolerance_ * 100);

    // カメラ内部パラメータをサブスクライブ (通常のQoSに変更)
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10, // <-- 通常のQoS(10)に変更
        std::bind(&ConeFusionNode::cameraInfoCallback, this,
                  std::placeholders::_1));

    // LiDARの候補点群をサブスクライブ
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cone_centers", 10,
        std::bind(&ConeFusionNode::lidarCallback, this, std::placeholders::_1));

    // カメラの色候補（ピクセル座標）をサブスクライブ
    color_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/color_regions", 10,
        std::bind(&ConeFusionNode::colorCallback, this, std::placeholders::_1));

    // 最終的に確定したコーンの位置をパブリッシュ
    confirmed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/confirmed_cones", 10);

    RCLCPP_INFO(this->get_logger(),
                "Fusion node started. Waiting for CameraInfo...");
  }

private:
  // カメラ情報が来たら、カメラモデルを初期化する
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!cam_model_.fromCameraInfo(msg)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera model");
    } else {
      cam_model_initialized_ = true;
      if (msg->header.frame_id.empty()) {
        camera_frame_id_ = "camera";
        RCLCPP_WARN(this->get_logger(),
                    "CameraInfo frame_id is empty. Using default 'camera'.");
      } else {
        camera_frame_id_ = msg->header.frame_id;
      }

      RCLCPP_INFO(this->get_logger(), "CameraInfo K[0]: %f, P[0]: %f",
                  msg->k[0], msg->p[0]);

      if (msg->k[0] == 0.0) {
        RCLCPP_WARN(
            this->get_logger(),
            "Camera uncalibrated! Using default parameters for 640x480.");
        auto mutable_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);

        mutable_msg->width = 640;
        mutable_msg->height = 480;

        // K行列 (3x3)
        mutable_msg->k[0] = 500.0; // fx
        mutable_msg->k[2] = 320.0; // cx
        mutable_msg->k[4] = 500.0; // fy
        mutable_msg->k[5] = 240.0; // cy
        mutable_msg->k[8] = 1.0;

        // P行列 (3x4)
        mutable_msg->p[0] = 500.0; // fx
        mutable_msg->p[2] = 320.0; // cx
        mutable_msg->p[5] = 500.0; // fy
        mutable_msg->p[6] = 240.0; // cy
        mutable_msg->p[10] = 1.0;

        cam_model_.fromCameraInfo(mutable_msg);
      } else {
        cam_model_.fromCameraInfo(msg);
      }

      if (!cam_model_.initialized()) { // check if initialization succeeded
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to initialize camera model even after fallback");
      } else {
        RCLCPP_INFO(this->get_logger(), "Camera model initialized!");
        cam_model_initialized_ = true;
        // 一度受信したらサブスクライバを停止
        camera_info_sub_.reset();
      }
    }
  }

  // カメラの色候補（ピクセル座標）を受信したら、最新のものを保持
  void colorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(),
                     "Received first color regions message!");
    latest_color_regions_ = msg;
  }

  // LiDARの候補点群がメインのトリガー
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg) {
    // 必要な情報（カメラモデル、色候補）がまだ来ていないなら何もしない
    if (!cam_model_initialized_ || !latest_color_regions_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for: %s %s",
                           cam_model_initialized_ ? "" : "CameraModel",
                           latest_color_regions_ ? "" : "ColorRegions");
      return;
    }

    // LiDARの点群 (PCL形式)
    pcl::PointCloud<pcl::PointXYZ> lidar_cloud;
    pcl::fromROSMsg(*lidar_msg, lidar_cloud);

    // カメラの色候補 (PCL形式、x,yにピクセル座標が入っている)
    pcl::PointCloud<pcl::PointXYZ> color_cloud;
    pcl::fromROSMsg(*latest_color_regions_, color_cloud);

    RCLCPP_INFO(this->get_logger(), "Lidar points: %zu, Color regions: %zu",
                lidar_cloud.points.size(), color_cloud.points.size());

    // TF取得: LiDARの座標系 (lidar_msg->header.frame_id) から
    //         カメラの座標系 (camera_frame_id_) への変換
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_.lookupTransform(
          camera_frame_id_,           // 変換先のフレーム (例: "camera_link")
          lidar_msg->header.frame_id, // 変換元のフレーム (例: "laser")
          tf2::TimePointZero,         // 最新のTFを取得
          std::chrono::milliseconds(100)); // 待機時間
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                  lidar_msg->header.frame_id.c_str(), camera_frame_id_.c_str(),
                  ex.what());
      return;
    }

    // LiDARの全候補点をループ処理
    for (const auto &lidar_point : lidar_cloud.points) {
      // 1. 3D座標をLiDAR座標系からカメラ座標系に変換
      geometry_msgs::msg::PointStamped point_in_lidar;
      point_in_lidar.header = lidar_msg->header;
      point_in_lidar.point.x = lidar_point.x;
      point_in_lidar.point.y = lidar_point.y;
      point_in_lidar.point.z = lidar_point.z;

      geometry_msgs::msg::PointStamped point_in_camera;
      tf2::doTransform(point_in_lidar, point_in_camera, tf_stamped);

      // 2. 3D座標（カメラ系）を2Dピクセル座標に投影
      cv::Point3d pt_3d(point_in_camera.point.x, point_in_camera.point.y,
                        point_in_camera.point.z);
      cv::Point2d pt_2d; // 投影されたピクセル座標(x, y)が入る

      // カメラの前方にある点（Z>0）だけを投影
      if (pt_3d.z > 0) {
        pt_2d = cam_model_.project3dToPixel(pt_3d);
      } else {
        RCLCPP_INFO(this->get_logger(), "Point behind camera: z=%f", pt_3d.z);
        continue; // カメラの後ろにある点は無視
      }

      // 3.
      // 投影されたピクセルが、色の候補のバウンディングボックス内にあるかチェック
      // さらに距離整合性もチェック
      bool found_color_match = false;

      // LiDARの実測距離（カメラ座標系でのz）
      double lidar_distance = pt_3d.z;

      // 全ての「色の候補点」をチェック
      // color_point.x, y = 重心座標, color_point.z = バウンディングボックス半幅
      for (const auto &color_point : color_cloud.points) {
        // ピクセル距離を計算
        double pixel_distance =
            std::hypot(pt_2d.x - color_point.x, pt_2d.y - color_point.y);

        // ステップ1: ピクセル距離チェック（閾値内か）
        if (pixel_distance < fusion_pixel_threshold_) {
          // ステップ2: 距離整合性チェック
          double half_size = color_point.z; // バウンディングボックスの半幅
          double focal_length = cam_model_.fx();
          double bbox_size_pixels = half_size * 2.0;

          // 期待距離 = (実サイズ * 焦点距離) / 画像上のサイズ
          double expected_distance =
              (cone_real_size_ * focal_length) / bbox_size_pixels;
          double distance_ratio = lidar_distance / expected_distance;

          RCLCPP_INFO(this->get_logger(),
                      "Checking: pixel_dist=%.1f, LiDAR=%.2fm, Expected=%.2fm, "
                      "Ratio=%.2f",
                      pixel_distance, lidar_distance, expected_distance,
                      distance_ratio);

          // 距離整合性チェック
          if (distance_ratio >= (1.0 - distance_tolerance_) &&
              distance_ratio <= (1.0 + distance_tolerance_)) {
            found_color_match = true;
            RCLCPP_INFO(this->get_logger(),
                        "MATCHED: pixel_dist=%.1f, ratio=%.2f", pixel_distance,
                        distance_ratio);
            break;
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "Distance mismatch: ratio=%.2f outside [%.2f, %.2f]",
                        distance_ratio, 1.0 - distance_tolerance_,
                        1.0 + distance_tolerance_);
          }
        }
      }

      // 4. 一致したら、追跡リストを更新
      if (found_color_match) {
        RCLCPP_DEBUG(
            this->get_logger(),
            "Match found! 3D(cam): [%.2f, %.2f, %.2f] -> 2D: [%.1f, %.1f]",
            pt_3d.x, pt_3d.y, pt_3d.z, pt_2d.x, pt_2d.y);

        // 既存の追跡候補に近いかチェック
        bool found_existing = false;
        for (auto &candidate : tracked_candidates_) {
          float dist =
              std::sqrt(std::pow(candidate.position.x - lidar_point.x, 2) +
                        std::pow(candidate.position.y - lidar_point.y, 2) +
                        std::pow(candidate.position.z - lidar_point.z, 2));
          if (dist < tracking_distance_) {
            // 既存の候補を更新
            candidate.detection_count++;
            candidate.last_detection = this->now();
            candidate.position = lidar_point; // 位置を更新
            found_existing = true;
            break;
          }
        }

        if (!found_existing) {
          // 新しい追跡候補を追加
          TrackedCandidate new_candidate;
          new_candidate.position = lidar_point;
          new_candidate.detection_count = 1;
          new_candidate.last_detection = this->now();
          tracked_candidates_.push_back(new_candidate);
        }
      }
    }

    // タイムアウトした候補を削除
    rclcpp::Time now = this->now();
    tracked_candidates_.erase(
        std::remove_if(tracked_candidates_.begin(), tracked_candidates_.end(),
                       [this, &now](const TrackedCandidate &c) {
                         return (now - c.last_detection).seconds() >
                                tracking_timeout_;
                       }),
        tracked_candidates_.end());

    // 確定回数に達した候補のみをパブリッシュ
    pcl::PointCloud<pcl::PointXYZ> confirmed_cloud;
    for (const auto &candidate : tracked_candidates_) {
      if (candidate.detection_count >= min_confirmation_count_) {
        confirmed_cloud.points.push_back(candidate.position);
      }
    }

    // 確定したコーンの点群をパブリッシュ
    if (!confirmed_cloud.points.empty()) {
      sensor_msgs::msg::PointCloud2 confirmed_msg;
      pcl::toROSMsg(confirmed_cloud, confirmed_msg);
      confirmed_msg.header = lidar_msg->header; // 元のフレームIDと時刻
      confirmed_pub_->publish(confirmed_msg);
    }
  }

  // TF用
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // カメラモデル用
  image_geometry::PinholeCameraModel cam_model_;
  bool cam_model_initialized_ = false;
  std::string camera_frame_id_;

  // サブスクライバ
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr color_sub_;

  // パブリッシャ
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr confirmed_pub_;

  // フュージョン閾値（ピクセル距離）
  double fusion_pixel_threshold_;

  // 連続検出カウンタ用
  int min_confirmation_count_;
  double tracking_distance_;
  double tracking_timeout_;
  std::vector<TrackedCandidate> tracked_candidates_;

  // 距離整合性チェック用
  double cone_real_size_;
  double distance_tolerance_;

  // 最新の色候補を保持する
  sensor_msgs::msg::PointCloud2::SharedPtr latest_color_regions_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeFusionNode>());
  rclcpp::shutdown();
  return 0;
}
