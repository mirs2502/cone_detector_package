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
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class PointCloudClusterNode : public rclcpp::Node {
public:
    PointCloudClusterNode()
        : Node("point_cloud_cluster_node"), last_processed_time_(this->now()) {
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", 10,
            std::bind(&PointCloudClusterNode::pointCloudCallback, this, std::placeholders::_1));

        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_point_cloud", 10);
        chair_legs_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/chair_legs", 10);

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
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // クラスタリング
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.05);  // 距離の閾値（50mm）
        ec.setMinClusterSize(3);      // クラスタ内の最小点数
        ec.setMaxClusterSize(30);     // クラスタ内の最大点数
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // フィルタリングされたクラスタを保持するための新しい点群
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // クラスタを収集し、重心を計算
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centers_cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
            cluster_centers_cloud->points.push_back(center);
        }

        // フィルタリングされた点群をROSメッセージに変換
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = current_point_cloud_->header;
        point_cloud_publisher_->publish(filtered_msg);

        // クラスタ重心をパブリッシュ
        sensor_msgs::msg::PointCloud2 cluster_centers_msg;
        pcl::toROSMsg(*cluster_centers_cloud, cluster_centers_msg);
        cluster_centers_msg.header = current_point_cloud_->header;
        chair_legs_publisher_->publish(cluster_centers_msg);

        last_processed_time_ = this->now(); // 最後の処理時間を更新
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr chair_legs_publisher_; // クラスタ重心パブリッシャー
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2::SharedPtr current_point_cloud_;
    rclcpp::Time last_processed_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudClusterNode>());
    rclcpp::shutdown();
    return 0;
}
