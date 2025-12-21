// Copyright 2024 MIRS2502
// Licensed under MIT License

#include "cone_detector/cone_costmap_layer.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cone_detector::ConeCostmapLayer, nav2_costmap_2d::Layer)

namespace cone_detector {

ConeCostmapLayer::ConeCostmapLayer()
    : cone_inflation_radius_(0.32), cone_topic_("/accumulated_cones"),
      cones_updated_(false), min_x_(0.0), min_y_(0.0), max_x_(0.0),
      max_y_(0.0) {}

ConeCostmapLayer::~ConeCostmapLayer() {}

void ConeCostmapLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in ConeCostmapLayer");
  }

  // パラメータの宣言と取得
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("cone_inflation_radius", rclcpp::ParameterValue(0.32));
  declareParameter("cone_topic", rclcpp::ParameterValue("/accumulated_cones"));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".cone_inflation_radius", cone_inflation_radius_);
  node->get_parameter(name_ + ".cone_topic", cone_topic_);

  // コストマップの初期化
  matchSize();
  current_ = true;

  // コーン位置のサブスクライバ
  cone_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      cone_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ConeCostmapLayer::coneCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
      node->get_logger(),
      "ConeCostmapLayer initialized: cone_inflation_radius=%.2f, cone_topic=%s",
      cone_inflation_radius_, cone_topic_.c_str());
}

void ConeCostmapLayer::coneCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cone_mutex_);

  // PCL形式に変換
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // コーン位置を更新
  cone_positions_.clear();
  for (const auto &pt : cloud->points) {
    geometry_msgs::msg::Point cone_pos;
    cone_pos.x = pt.x;
    cone_pos.y = pt.y;
    cone_pos.z = pt.z;
    cone_positions_.push_back(cone_pos);
  }

  cones_updated_ = true;
}

void ConeCostmapLayer::updateBounds(double /*robot_x*/, double /*robot_y*/,
                                    double /*robot_yaw*/, double *min_x,
                                    double *min_y, double *max_x,
                                    double *max_y) {
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(cone_mutex_);

  if (cone_positions_.empty()) {
    return;
  }

  // 全コーン位置を含む領域を計算
  min_x_ = std::numeric_limits<double>::max();
  min_y_ = std::numeric_limits<double>::max();
  max_x_ = std::numeric_limits<double>::lowest();
  max_y_ = std::numeric_limits<double>::lowest();

  for (const auto &cone : cone_positions_) {
    min_x_ = std::min(min_x_, cone.x - cone_inflation_radius_);
    min_y_ = std::min(min_y_, cone.y - cone_inflation_radius_);
    max_x_ = std::max(max_x_, cone.x + cone_inflation_radius_);
    max_y_ = std::max(max_y_, cone.y + cone_inflation_radius_);
  }

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);
}

void ConeCostmapLayer::setConeInflation(double wx, double wy) {
  // コーン位置を中心に膨張半径内のセルにコストを設定
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    return;
  }

  // 膨張半径をセル数に変換
  unsigned int cell_radius = static_cast<unsigned int>(
      std::ceil(cone_inflation_radius_ / resolution_));

  // 膨張半径内の各セルにコストを設定
  for (int dx = -static_cast<int>(cell_radius);
       dx <= static_cast<int>(cell_radius); ++dx) {
    for (int dy = -static_cast<int>(cell_radius);
         dy <= static_cast<int>(cell_radius); ++dy) {
      int nx = static_cast<int>(mx) + dx;
      int ny = static_cast<int>(my) + dy;

      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x_) ||
          ny >= static_cast<int>(size_y_)) {
        continue;
      }

      // 距離を計算
      double dist = std::sqrt(dx * dx + dy * dy) * resolution_;

      if (dist <= cone_inflation_radius_) {
        // コーン中心は致命的コスト、外側に向かって減衰
        unsigned char cost;
        if (dist < resolution_) {
          // コーン中心
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        } else {
          // 距離に応じてコストを減衰（線形減衰）
          double ratio = dist / cone_inflation_radius_;
          cost = static_cast<unsigned char>(nav2_costmap_2d::LETHAL_OBSTACLE *
                                            (1.0 - ratio));
          // 最低でもINSCRIBED未満に
          cost = std::min(
              cost, static_cast<unsigned char>(
                        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1));
        }

        // 既存のコストと比較して高い方を採用
        unsigned char current_cost = getCost(nx, ny);
        if (cost > current_cost) {
          setCost(nx, ny, cost);
        }
      }
    }
  }
}

void ConeCostmapLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                   int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(cone_mutex_);

  // コーン周辺のコストをクリア（通常の障害物レイヤーの膨張を上書きするため）
  for (const auto &cone : cone_positions_) {
    setConeInflation(cone.x, cone.y);
  }

  // マスターグリッドに反映
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void ConeCostmapLayer::reset() {
  resetMaps();
  std::lock_guard<std::mutex> lock(cone_mutex_);
  cone_positions_.clear();
  cones_updated_ = false;
}

} // namespace cone_detector
