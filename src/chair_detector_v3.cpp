#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::PoseStamped;

class ChairRecognitionNode : public rclcpp::Node {
public:
    ChairRecognitionNode() : Node("chair_recognition"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // 椅子の脚候補のサブスクライブ
        pointcloud_subscriber_ = this->create_subscription<PointCloud2>(
            "chair_legs", 10, std::bind(&ChairRecognitionNode::legsCallback, this, std::placeholders::_1)
        );

        // 椅子の位置姿勢のパブリッシュ
        chair_publisher_ = this->create_publisher<PoseStamped>("chair_pose", 10);
    }

private:
    void legsCallback(const PointCloud2::SharedPtr msg) {
        // PointCloud2をpcl::PointCloudに変換
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // 脚の位置を抽出
        std::vector<std::pair<double, double>> leg_positions;
        for (const auto &point : pcl_cloud.points) {
            leg_positions.emplace_back(point.x, point.y);
        }

        // 3つ以上の脚を持つクラスターを見つける
        auto chair_clusters = findChairClusters(leg_positions);

        // 認識された椅子の姿勢をパブリッシュ
        for (const auto &cluster : chair_clusters) {
            auto pose = calculatePose(cluster);
            transformPoseToMapFrame(pose);
            chair_publisher_->publish(pose);
        }
    }

    std::vector<std::vector<std::pair<double, double>>> findChairClusters(const std::vector<std::pair<double, double>> &leg_positions) {
        std::vector<std::vector<std::pair<double, double>>> clusters;

        // 40cm以内にある脚をグループ化
        for (size_t i = 0; i < leg_positions.size(); ++i) {
            for (size_t j = i + 1; j < leg_positions.size(); ++j) {
                for (size_t k = j + 1; k < leg_positions.size(); ++k) {
                    for (size_t l = k + 1; l < leg_positions.size(); ++l) {
                        std::vector<std::pair<double, double>> quad = {leg_positions[i], leg_positions[j], leg_positions[k], leg_positions[l]};
                        if (isValidSquare(quad)) {
                            clusters.push_back(quad);
                        }
                    }
                }
            }
        }

        return clusters;
    }

    bool isValidSquare(const std::vector<std::pair<double, double>> &legs) {
        std::vector<double> distances;
        for (size_t i = 0; i < legs.size(); ++i) {
            for (size_t j = i + 1; j < legs.size(); ++j) {
                distances.push_back(std::hypot(legs[i].first - legs[j].first, legs[i].second - legs[j].second));
            }
        }

        // 距離をソートして正方形の特性をチェック
        std::sort(distances.begin(), distances.end());
        double side = 0.45; // 40cmをメートルに変換
        return (
            std::abs(distances[0] - side) <= 0.05 &&
            std::abs(distances[1] - side) <= 0.05 &&
            std::abs(distances[2] - side) <= 0.05 &&
            std::abs(distances[3] - side) <= 0.05 &&
            std::abs(distances[4] - side * std::sqrt(2)) <= 0.05
        );
    }

    PoseStamped calculatePose(const std::vector<std::pair<double, double>> &cluster) {
        // 中心を計算
        double x_sum = 0.0, y_sum = 0.0;
        for (const auto &pos : cluster) {
            x_sum += pos.first;
            y_sum += pos.second;
        }
        double center_x = x_sum / cluster.size();
        double center_y = y_sum / cluster.size();

        // 姿勢を計算（クラスター内の隣り合う脚同士の間を合計4回計算）
        double best_yaw = 0.0;
        double min_angle_diff = std::numeric_limits<double>::max();
        for (size_t i = 0; i < cluster.size(); ++i) {
            size_t j = (i + 1) % cluster.size();
            double dx = cluster[j].first - cluster[i].first;
            double dy = cluster[j].second - cluster[i].second;
            double yaw = std::atan2(dy, dx);
            double angle_diff = std::abs(yaw);
            if (angle_diff < min_angle_diff) {
            min_angle_diff = angle_diff;
            best_yaw = yaw;
            }
        }
        double yaw = best_yaw;

        // 最終的にパブリッシュする位置は椅子の中心よりも50cm後ろにする
        center_x -= 0.5 * std::cos(yaw);

        // PoseStampedを作成
        PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = center_x;
        pose.pose.position.y = center_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);

        return pose;
    }

    void transformPoseToMapFrame(PoseStamped &pose) {
        try {
            // tf2を使ってbase_linkからmapへの変換を取得
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            // 変換を適用
            tf2::doTransform(pose, pose, transform_stamped);
            pose.header.frame_id = "map";
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform pose to map frame: %s", ex.what());
        }
    }

    rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<PoseStamped>::SharedPtr chair_publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChairRecognitionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}