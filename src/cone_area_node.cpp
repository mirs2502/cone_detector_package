#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/geometry.h>

// TF2 (座標変換) 用ヘッダ
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> // PointCloud2の変換用

class ConeAreaNode : public rclcpp::Node {
public:
    ConeAreaNode() : Node("cone_area_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        
        // パラメータ: 蓄積する固定座標系の名前 (通常は "odom" か "map")
        // SLAMを使っているなら "map", オドメトリのみなら "odom" 推奨
        this->declare_parameter("target_frame", "odom");
        this->get_parameter("target_frame", target_frame_);

        centers_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/confirmed_cones",
            10,
            std::bind(&ConeAreaNode::centersCallback, this, std::placeholders::_1));

        polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/cone_area", 10);
        accumulated_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/accumulated_cones", 10);
        
        // 記憶用クラウドの初期化
        accumulated_cones_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

private:
    void centersCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        
        // 1. 座標変換: 入力点群を target_frame (map/odom) に変換する
        sensor_msgs::msg::PointCloud2 transformed_msg;
        try {
            // 最新のTFではなく、点群が計測された時刻(msg->header.stamp)のTFを使うのが重要
            // タイムアウト時間を少し設ける
            geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
                target_frame_, msg->header.frame_id, 
                msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

            tf2::doTransform(*msg, transformed_msg, tf_stamped);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Transform failure: %s", ex.what());
            return;
        }

        // 2. PCL形式に変換 (変換後のデータを使用)
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_msg, *current_cloud);

        bool updated = false;

        // 3. 記憶済みのコーンと比較して、新しいものだけ追加 (Accumulation)
        for (const auto& new_pt : current_cloud->points) {
            if (isNewCone(new_pt)) {
                accumulated_cones_->points.push_back(new_pt);
                updated = true;
            }
        }

        // デバッグ用: 蓄積されたコーン点群をパブリッシュ
        if (accumulated_cones_->points.size() > 0) {
            sensor_msgs::msg::PointCloud2 accumulated_msg;
            pcl::toROSMsg(*accumulated_cones_, accumulated_msg);
            accumulated_msg.header.frame_id = target_frame_;
            accumulated_msg.header.stamp = this->now();
            accumulated_publisher_->publish(accumulated_msg);
        }

        // 4. コーンが3つ以上あれば凸包（エリア）を計算
        if (accumulated_cones_->points.size() < 3) {
            // まだ作成できないが、データは蓄積中
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        
        chull.setInputCloud(accumulated_cones_);
        chull.setDimension(2); // 2D凸包
        chull.reconstruct(*cloud_hull);

        // 5. PolygonStamped メッセージ作成
        geometry_msgs::msg::PolygonStamped polygon_msg;
        polygon_msg.header.frame_id = target_frame_; // ヘッダは固定座標系にする
        polygon_msg.header.stamp = this->now();

        for (const auto& point : cloud_hull->points) {
            geometry_msgs::msg::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            polygon_msg.polygon.points.push_back(p);
        }

        polygon_publisher_->publish(polygon_msg);
        
        if (updated) {
            RCLCPP_INFO(this->get_logger(), "Area updated. Total cones: %zu, Polygon points: %zu", 
                        accumulated_cones_->points.size(), polygon_msg.polygon.points.size());
        }
    }

    // 重複チェック: 既存のコーンから一定距離(例: 0.3m)以内なら同一とみなす
    bool isNewCone(const pcl::PointXYZ& new_pt) {
        const double threshold_dist = 0.3; 
        for (const auto& existing_pt : accumulated_cones_->points) {
            float dist = pcl::geometry::distance(new_pt, existing_pt);
            if (dist < threshold_dist) {
                return false; // 既知のコーン
            }
        }
        return true; // 新しいコーン
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr centers_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_publisher_;
    
    // TF関連
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string target_frame_;

    // 蓄積用データ
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cones_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeAreaNode>());
    rclcpp::shutdown();
    return 0;
}
