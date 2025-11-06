#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp> // 多角形メッセージ
#include <geometry_msgs/msg/point32.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h> // 凸包計算用
#include <pcl/common/io.h> // for copyPointCloud

class ConeAreaNode : public rclcpp::Node {
public:
    ConeAreaNode() : Node("cone_area_node") {
        centers_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/confirmed_cones", // フュージョンノードの出力に変更
            10,
            std::bind(&ConeAreaNode::centersCallback, this, std::placeholders::_1));

        polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/cone_area", 10);
    }

private:
    void centersCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centers(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_centers);

        // コーンが3つ以上ないと多角形（凸包）が作れない
        if (cloud_centers->points.size() < 3) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for 3 or more cone centers to compute convex hull.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        
        chull.setInputCloud(cloud_centers);
        chull.setDimension(2); // X-Y平面での2D凸包を指定
        chull.reconstruct(*cloud_hull);

        // 凸包の頂点群を PolygonStamped メッセージとして作成
        geometry_msgs::msg::PolygonStamped polygon_msg;
        polygon_msg.header = msg->header; // フレームIDとタイムスタンプを引き継ぐ

        for (const auto& point : cloud_hull->points) {
            geometry_msgs::msg::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z; // 凸包のZ座標
            polygon_msg.polygon.points.push_back(p);
        }

        polygon_publisher_->publish(polygon_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr centers_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeAreaNode>());
    rclcpp::shutdown();
    return 0;
}
