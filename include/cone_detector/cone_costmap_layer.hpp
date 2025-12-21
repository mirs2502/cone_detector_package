// Copyright 2024 MIRS2502
// Licensed under MIT License

#ifndef CONE_DETECTOR__CONE_COSTMAP_LAYER_HPP_
#define CONE_DETECTOR__CONE_COSTMAP_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace cone_detector {

/**
 * @class ConeCostmapLayer
 * @brief
 * コーン周辺のコストを低く（膨張半径を小さく）設定するカスタムコストマップレイヤー
 */
class ConeCostmapLayer : public nav2_costmap_2d::CostmapLayer {
public:
  ConeCostmapLayer();
  virtual ~ConeCostmapLayer();

  /**
   * @brief プラグインの初期化
   */
  virtual void onInitialize() override;

  /**
   * @brief 更新が必要な領域を計算
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double *min_x, double *min_y, double *max_x,
                            double *max_y) override;

  /**
   * @brief コストマップにコストを反映
   */
  virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i,
                           int min_j, int max_i, int max_j) override;

  /**
   * @brief レイヤーのリセット
   */
  virtual void reset() override;

  /**
   * @brief このレイヤーがクリアリングをサポートするか
   */
  virtual bool isClearable() override { return false; }

private:
  /**
   * @brief コーン位置のコールバック
   */
  void coneCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief 指定位置の周囲にコストを設定
   */
  void setConeInflation(double wx, double wy);

  // パラメータ
  double cone_inflation_radius_; // コーン周辺の膨張半径（デフォルト: 0.32m）
  std::string cone_topic_;       // コーン位置のトピック名

  // コーン位置の保持
  std::vector<geometry_msgs::msg::Point> cone_positions_;
  std::mutex cone_mutex_;

  // サブスクライバ
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cone_sub_;

  // 更新フラグ
  bool cones_updated_;

  // 更新領域
  double min_x_, min_y_, max_x_, max_y_;
};

} // namespace cone_detector

#endif // CONE_DETECTOR__CONE_COSTMAP_LAYER_HPP_
