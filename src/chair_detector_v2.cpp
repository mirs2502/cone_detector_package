#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <cmath>
#include <algorithm>

using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::PoseStamped;

class ChairRecognitionNode : public rclcpp::Node {
public:
    ChairRecognitionNode() : Node("chair_recognition") {
        pointcloud_subscriber_ = this->create_subscription<PointCloud2>(
            "chair_legs", 10, std::bind(&ChairRecognitionNode::legsCallback, this, std::placeholders::_1)
        );
        chair_publisher_ = this->create_publisher<PoseStamped>("chair_pose", 10);
    }

private:
    void legsCallback(const PointCloud2::SharedPtr msg) {
        // Convert PointCloud2 to pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Extract leg positions
        std::vector<std::pair<double, double>> leg_positions;
        for (const auto &point : pcl_cloud.points) {
            leg_positions.emplace_back(point.x, point.y);
        }

        // Find clusters of legs with 3 or more legs
        auto chair_clusters = findChairClusters(leg_positions);

        // Publish the recognized chair pose
        for (const auto &cluster : chair_clusters) {
            auto pose = calculatePose(cluster);
            chair_publisher_->publish(pose);
        }
    }

    std::vector<std::vector<std::pair<double, double>>> findChairClusters(const std::vector<std::pair<double, double>> &leg_positions) {
        std::vector<std::vector<std::pair<double, double>>> clusters;

        // Group legs that are within 40cm of each other
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

        // Sort distances and check square properties
        std::sort(distances.begin(), distances.end());
        double side = 0.45; // 40cm in meters
        return (
            std::abs(distances[0] - side) <= 0.05 &&
            std::abs(distances[1] - side) <= 0.05 &&
            std::abs(distances[2] - side) <= 0.05 &&
            std::abs(distances[3] - side) <= 0.05 &&
            std::abs(distances[4] - side * std::sqrt(2)) <= 0.05
        );
    }

    PoseStamped calculatePose(const std::vector<std::pair<double, double>> &cluster) {
        // Calculate center
        double x_sum = 0.0, y_sum = 0.0;
        for (const auto &pos : cluster) {
            x_sum += pos.first;
            y_sum += pos.second;
        }
        double center_x = x_sum / cluster.size();
        double center_y = y_sum / cluster.size();

        // Calculate orientation (from first leg to second leg in cluster)
        double dx = cluster[1].first - cluster[0].first;
        double dy = cluster[1].second - cluster[0].second;
        double yaw = std::atan2(dy, dx);

        // Create PoseStamped
        PoseStamped pose;
        pose.header.frame_id = "map";
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

    rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<PoseStamped>::SharedPtr chair_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChairRecognitionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
