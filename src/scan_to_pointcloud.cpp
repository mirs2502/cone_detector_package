#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

// TFリスナーに必要なヘッダ
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp" // (LaserScanのTF変換に使う)

// unique_ptr, shared_ptr を使うために必要
#include <memory>

class ScanToPointCloudNode : public rclcpp::Node
{
public:
    // TFリスナーの初期化（★ C++エラーを修正 ★）
    ScanToPointCloudNode() : Node("scan_to_pointcloud_node"),
        // 1. Clock を渡して Buffer を初期化
        tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
        // 2. Buffer を渡して Listener を初期化
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToPointCloudNode::scanCallback, this, std::placeholders::_1));

        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    }

private:
void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        try
        {
            // 1. ターゲットフレームを定義
            std::string target_frame = "base_link"; //12月10日odomから変更
            
            // 2. laser_geometryにTFリスナーを渡して変換させる
            // (canTransformチェックを削除し、この関数にTF待機を任せる)
            sensor_msgs::msg::PointCloud2 cloud_msg;
            projector_.transformLaserScanToPointCloud(
                target_frame, *scan_msg, cloud_msg, *tf_buffer_);
            
            // 3. 変換後のPointCloud2をパブリッシュ
            cloud_publisher_->publish(cloud_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "scan_to_pointcloud: TF exception: %s", ex.what());
        }
    }

    // TFリスナーとバッファ（★ C++エラーを修正 ★）
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
    // LaserScanからPointCloud2への変換を行うオブジェクト
    laser_geometry::LaserProjection projector_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToPointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
