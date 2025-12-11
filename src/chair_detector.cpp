#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <cmath>

class ChairDetectionNode : public rclcpp::Node
{
public:
    ChairDetectionNode() : Node("chair_detection")
    {
        // サブスクライバー設定
        subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/cluster_centers", 10,
            std::bind(&ChairDetectionNode::cluster_centers_callback, this, std::placeholders::_1));

        // パブリッシャー設定
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/chair_positions", 10);

        // 距離閾値の設定
        distance_min_ = 0.2;  // 最小距離
        distance_max_ = 0.6;  // 最大距離
    }

private:
    void cluster_centers_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        // メッセージ内のクラスタの重心座標を抽出
        std::vector<std::pair<double, double>> centroids;
        for (const auto &marker : msg->markers) {
            centroids.push_back({marker.pose.position.x, marker.pose.position.y});
        }

        // クラスタ数が3つ以上なければ処理しない
        if (centroids.size() < 3) {
            return;
        }

        // 距離行列の計算
        std::vector<std::vector<double>> dist_matrix = calculate_distance_matrix(centroids);

        // 椅子の足候補を探す
        std::vector<std::vector<int>> detected_chairs;
        for (size_t i = 0; i < dist_matrix.size(); ++i) {
            std::vector<int> neighbors;
            for (size_t j = 0; j < dist_matrix[i].size(); ++j) {
                if (dist_matrix[i][j] > distance_min_ && dist_matrix[i][j] < distance_max_) {
                    neighbors.push_back(j);
                }
            }
            if (neighbors.size() >= 2 && neighbors.size() <= 3) {
                detected_chairs.push_back(neighbors);
            }
        }

        // 椅子の足候補が4つ以上であれば、4本足を仮定して重心を計算
        visualization_msgs::msg::MarkerArray chair_markers;
        int id = 0;
        for (const auto &chair : detected_chairs) {
            if (chair.size() >= 3) {
                // 重心計算（仮定して4点目を計算）
                std::vector<std::pair<double, double>> chair_positions = {
                    centroids[chair[0]],
                    centroids[chair[1]],
                    centroids[chair[2]]
                };

                // 椅子の4本目の足を予測
                double avg_x = (chair_positions[0].first + chair_positions[1].first) / 2;
                double avg_y = (chair_positions[0].second + chair_positions[1].second) / 2;
                chair_positions.push_back({avg_x, avg_y});  // 予測した点

                // 4本足の重心を計算
                double center_x = 0.0;
                double center_y = 0.0;
                for (const auto &pos : chair_positions) {
                    center_x += pos.first;
                    center_y += pos.second;
                }
                center_x /= chair_positions.size();
                center_y /= chair_positions.size();

                // 椅子の位置をマーカとして作成
                visualization_msgs::msg::Marker chair_center_marker;
                chair_center_marker.header.frame_id = "laser";  // フレームIDを適切に設定
                chair_center_marker.header.stamp = this->get_clock()->now();
                chair_center_marker.ns = "chair_center";
                chair_center_marker.id = id++;
                chair_center_marker.type = visualization_msgs::msg::Marker::SPHERE;
                chair_center_marker.action = visualization_msgs::msg::Marker::ADD;
                chair_center_marker.pose.position.x = center_x;
                chair_center_marker.pose.position.y = center_y;
                chair_center_marker.pose.position.z = 0.0;
                chair_center_marker.pose.orientation.w = 1.0;
                chair_center_marker.scale.x = 0.1;
                chair_center_marker.scale.y = 0.1;
                chair_center_marker.scale.z = 0.1;
                chair_center_marker.color.r = 1.0;
                chair_center_marker.color.g = 1.0;
                chair_center_marker.color.b = 0.0;
                chair_center_marker.color.a = 1.0;
                chair_center_marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(0.5));
                chair_markers.markers.push_back(chair_center_marker);
            }
        }

        // 椅子の位置マーカをパブリッシュ
        publisher_->publish(chair_markers);
    }

    std::vector<std::vector<double>> calculate_distance_matrix(const std::vector<std::pair<double, double>> &centroids)
    {
        std::vector<std::vector<double>> dist_matrix(centroids.size(), std::vector<double>(centroids.size()));
        for (size_t i = 0; i < centroids.size(); ++i) {
            for (size_t j = 0; j < centroids.size(); ++j) {
                dist_matrix[i][j] = std::sqrt(std::pow(centroids[i].first - centroids[j].first, 2) +
                                              std::pow(centroids[i].second - centroids[j].second, 2));
            }
        }
        return dist_matrix;
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    double distance_min_;
    double distance_max_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChairDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
