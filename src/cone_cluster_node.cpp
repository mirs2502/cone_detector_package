#include <memory>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h> // クラスタリング用
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

// ★ 円モデル（RANSAC）のために追加 ★
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

class ConeClusterNode : public rclcpp::Node {
public:
  ConeClusterNode() : Node("cone_cluster_node") {

    // クラスタリング用パラメータ
    this->declare_parameter("cluster_tolerance", 0.25);
    this->declare_parameter("min_cluster_size", 10);
    this->declare_parameter("max_cluster_size", 500);

    // ★ 円モデル（RANSAC）用パラメータ ★
    this->declare_parameter("circle_dist_thresh",
                            0.05); // 5cm (円からこの距離までを許容)
    this->declare_parameter("circle_min_radius",
                            0.05); // 5cm (コーンの最小半径)
    this->declare_parameter("circle_max_radius",
                            0.3); // 30cm (コーンの最大半径)
    this->declare_parameter("circle_inlier_ratio",
                            0.7); // 70% (クラスタの何%が円に適合すべきか)

    update_parameters();

    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &ConeClusterNode::parameterCallback, this, std::placeholders::_1));

    point_cloud_subscription_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", 10,
            std::bind(&ConeClusterNode::pointCloudCallback, this,
                      std::placeholders::_1));

    cone_centers_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/cone_centers",
                                                              10);
  }

private:
  rcl_interfaces::msg::SetParametersResult
  parameterCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
      if (param.get_name() == "cluster_tolerance")
        cluster_tolerance_ = param.as_double();
      if (param.get_name() == "min_cluster_size")
        min_cluster_size_ = param.as_int();
      if (param.get_name() == "max_cluster_size")
        max_cluster_size_ = param.as_int();
      if (param.get_name() == "circle_dist_thresh")
        circle_dist_thresh_ = param.as_double();
      if (param.get_name() == "circle_min_radius")
        circle_min_radius_ = param.as_double();
      if (param.get_name() == "circle_max_radius")
        circle_max_radius_ = param.as_double();
      if (param.get_name() == "circle_inlier_ratio")
        circle_inlier_ratio_ = param.as_double();
    }
    return result;
  }

  void update_parameters() {
    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("circle_dist_thresh", circle_dist_thresh_);
    this->get_parameter("circle_min_radius", circle_min_radius_);
    this->get_parameter("circle_max_radius", circle_max_radius_);
    this->get_parameter("circle_inlier_ratio", circle_inlier_ratio_);
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    std::vector<int> indices_nan;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices_nan);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centers_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // ★ RANSAC（円モデル）の準備 ★
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE2D); // 2Dの円モデル
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(circle_dist_thresh_); // 許容距離
    seg.setMaxIterations(100);

    // 検出した全クラスタをループ
    for (const auto &cluster : cluster_indices) {

      // 1. クラスタの点群を抽出
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (const int index : cluster.indices) {
        cloud_cluster->points.push_back(cloud->points[index]);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // 2. ★ そのクラスタに円モデルを当てはめる ★
      seg.setInputCloud(cloud_cluster);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) {
        // 円が見つからなかった（直線 など）場合はスキップ
        continue;
      }

      // 3. ★ 円の半径をチェック ★
      float radius = coefficients->values[2];
      if (radius < circle_min_radius_ || radius > circle_max_radius_) {
        // 半径がコーンのサイズと違う場合はスキップ
        continue;
      }

      // 4. ★ inlier比率をチェック（追加）★
      double inlier_ratio = static_cast<double>(inliers->indices.size()) /
                            static_cast<double>(cloud_cluster->points.size());
      if (inlier_ratio < circle_inlier_ratio_) {
        // 円に適合した点が少なすぎる場合はスキップ（円弧らしくない）
        continue;
      }

      // 4. ★ 条件（円であり、半径が適切）をクリアしたクラスタの重心を計算 ★
      pcl::PointXYZ center;
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

    // フィルタリングされた「円弧」の重心だけをパブリッシュ
    sensor_msgs::msg::PointCloud2 cluster_centers_msg;
    pcl::toROSMsg(*cluster_centers_cloud, cluster_centers_msg);
    cluster_centers_msg.header = msg->header;
    cone_centers_publisher_->publish(cluster_centers_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cone_centers_publisher_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  // ★ RANSAC用パラメータ変数 ★
  double circle_dist_thresh_;
  double circle_min_radius_;
  double circle_max_radius_;
  double circle_inlier_ratio_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeClusterNode>());
  rclcpp::shutdown();
  return 0;
}
