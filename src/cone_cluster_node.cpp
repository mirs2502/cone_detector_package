#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h> // クラスタリング用

// ノード名を変更
class ConeClusterNode : public rclcpp::Node {
public:
    ConeClusterNode()
        : Node("cone_cluster_node") { // ノード名を変更
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", // scan_to_pointcloudノードの出力
            10,
            std::bind(&ConeClusterNode::pointCloudCallback, this, std::placeholders::_1));

        // 出力トピック名を変更
        cone_centers_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cone_centers", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // （オプション：NaN点（無効な点）を除去）
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        
        // --- カラーコーン用のパラメータ調整 (要調整) ---
        ec.setClusterTolerance(0.25);  // 25cm (この距離内の点を同一クラスタとみなす)
        ec.setMinClusterSize(10);      // カラーコーン1つを構成する最小の点群数
        ec.setMaxClusterSize(500);     // カラーコーン1つを構成する最大の点群数
        // ------------------------------------------

        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // クラスタの重心を格納する新しいPointCloudを作成
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centers_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& cluster : cluster_indices) {
            pcl::PointXYZ center;
            // クラスタ内の全点の平均を計算して重心を求める
            for (int index : cluster.indices) {
                center.x += cloud->points[index].x;
                center.y += cloud->points[index].y;
                center.z += cloud->points[index].z;
            }
            center.x /= cluster.indices.size();
            center.y /= cluster.indices.size();
            center.z /= cluster.indices.size();
            cluster_centers_cloud->points.push_back(center);
        }

        // クラスタ重心群をPointCloud2メッセージとしてパブリッシュ
        sensor_msgs::msg::PointCloud2 cluster_centers_msg;
        pcl::toROSMsg(*cluster_centers_cloud, cluster_centers_msg);
        cluster_centers_msg.header = msg->header; // 元の点群のフレームIDとタイムスタンプを引き継ぐ
        cone_centers_publisher_->publish(cluster_centers_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cone_centers_publisher_; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeClusterNode>());
    rclcpp::shutdown();
    return 0;
}
