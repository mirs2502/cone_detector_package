#include <memory>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h> // EuclideanClusterExtractionのため
#include <visualization_msgs/msg/marker_array.hpp>

class PointCloudClusterNode : public rclcpp::Node {
public:
    PointCloudClusterNode()
        : Node("point_cloud_cluster_node"), last_processed_time_(this->now()) {
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", 10,
            std::bind(&PointCloudClusterNode::pointCloudCallback, this, std::placeholders::_1));

        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_point_cloud", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/chair_legs", 10); // マーカー用パブリッシャー追加

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100msごとに処理を実行
            std::bind(&PointCloudClusterNode::processPointClouds, this));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        current_point_cloud_ = msg; // 現在の点群データを保持
    }

    void processPointClouds() {
        if (!current_point_cloud_) {
            return; // 現在の点群データがない場合は処理をスキップ
        }

        // 点群データをPCL形式に変換
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*current_point_cloud_, *cloud);

        // KD-Treeの作成
        typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
        typename KdTree::Ptr tree(new KdTree);
        //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // DBSCANクラスタリング
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.05);  // 距離の閾値（50mm）
        ec.setMinClusterSize(3);        // クラスタ内の最小点数
        ec.setMaxClusterSize(30);      // クラスタ内の最大点数
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // フィルタリングされたクラスタを保持するための新しい点群
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // クラスタを収集し、重心を計算
        std::vector<pcl::PointXYZ> cluster_centers;
        for (const auto& indices : cluster_indices) {
            if (indices.indices.size() < 3 || indices.indices.size() > 100) {
                continue; // クラスタのサイズが指定の範囲外の場合はスキップ
            }

            pcl::PointXYZ center;
            for (int index : indices.indices) {
                filtered_cloud->points.push_back(cloud->points[index]);
                center.x += cloud->points[index].x;
                center.y += cloud->points[index].y;
                center.z += cloud->points[index].z;
            }
            center.x /= indices.indices.size();
            center.y /= indices.indices.size();
            center.z /= indices.indices.size();
            cluster_centers.push_back(center);
        }

        // フレーム間連続性のためのクラスタ情報を更新
        updateClusterContinuity(cluster_centers);

        // フィルタリングされた点群をROSメッセージに変換
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header = current_point_cloud_->header;
        point_cloud_publisher_->publish(output_msg);

        publishClusterCenters(cluster_centers);

        last_processed_time_ = this->now(); // 最後の処理時間を更新
    }

    void updateClusterContinuity(const std::vector<pcl::PointXYZ>& cluster_centers) {
        // 前のフレームのクラスタ情報を確認
        for (const auto& current_center : cluster_centers) {
            bool found = false;
            for (const auto& previous_center : previous_cluster_centers_) {
                // 距離が閾値以内であれば、同じクラスタとみなす
                float distance = std::sqrt(
                    std::pow(current_center.x - previous_center.x, 2) +
                    std::pow(current_center.y - previous_center.y, 2) +
                    std::pow(current_center.z - previous_center.z, 2));
                if (distance < 0.2) { // 10cmの閾値で比較
                    found = true;
                    break;
                }
            }
            if (!found) {
                RCLCPP_INFO(this->get_logger(), "New cluster detected at (%.2f, %.2f, %.2f)", current_center.x, current_center.y, current_center.z);
            }
        }

        // 前のフレームのクラスタ情報を更新
        previous_cluster_centers_ = cluster_centers;
    }

    void publishClusterCenters(const std::vector<pcl::PointXYZ>& cluster_centers) {
        visualization_msgs::msg::MarkerArray marker_array;

        int cluster_id = 0;
        for (const auto& center : cluster_centers) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser"; // 適切なフレームを指定
            marker.header.stamp = this->now();
            marker.id = cluster_id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = center.x;
            marker.pose.position.y = center.y;
            marker.pose.position.z = center.z;
            marker.scale.x = 0.1; // 重心のサイズ
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0f; // 赤色
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0; // 不透明度
            marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(0.5));

            marker_array.markers.push_back(marker);
            cluster_id++;
        }

        marker_publisher_->publish(marker_array); // マーカーをRVizに公開
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2::SharedPtr current_point_cloud_;
    rclcpp::Time last_processed_time_;

    std::vector<pcl::PointXYZ> previous_cluster_centers_; // 前回のクラスタの重心

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_; // マーカーパブリッシャー
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudClusterNode>());
    rclcpp::shutdown();
    return 0;
}
