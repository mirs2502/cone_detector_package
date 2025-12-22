#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.hpp> // 削除
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// #defineでのハードコードを削除

class ConeColorDetectorNode : public rclcpp::Node {
public:
  ConeColorDetectorNode() : Node("cone_color_detector_node") {
    // パラメータを宣言 (H: 0-179, S: 0-255, V: 0-255)
    // 赤色コーン用 (Hueが0付近と180付近にまたがるため、2つの範囲が必要だが、
    // ここでは簡易的に0-10と160-180の広い範囲をカバーするように設定、または2つのフィルタを組み合わせる必要がある)
    // 今回はとりあえずオレンジ〜赤を広めに取る設定に変更
    // パラメータを宣言 (H: 0-179, S: 0-255, V: 0-255)
    // 赤色コーン用 (Hueが0付近と180付近にまたがるため、2つの範囲を設定)
    
    // 範囲1: オレンジ〜赤 (0-15)
    this->declare_parameter("h_min", 0);
    this->declare_parameter("h_max", 15);
    
    // 範囲2: 紫〜赤 (165-180)
    this->declare_parameter("h_min_2", 165);
    this->declare_parameter("h_max_2", 180);

    this->declare_parameter("s_min", 100); // 30 -> 100: 白っぽい色を除外
    this->declare_parameter("s_max", 255);
    
    this->declare_parameter("v_min", 50); // 30 -> 50: 暗すぎる色を除外
    this->declare_parameter("v_max", 255);

    // 宣言したパラメータを変数に読み込む
    this->get_parameter("h_min", h_min_);
    this->get_parameter("h_max", h_max_);
    this->get_parameter("h_min_2", h_min_2_);
    this->get_parameter("h_max_2", h_max_2_);
    this->get_parameter("s_min", s_min_);
    this->get_parameter("s_max", s_max_);
    this->get_parameter("v_min", v_min_);
    this->get_parameter("v_max", v_max_);

    // 面積フィルタのパラメータ
    this->declare_parameter("min_area", 500);   // 最小面積(ピクセル²)
    this->declare_parameter("max_area", 50000); // 最大面積(ピクセル²)
    this->get_parameter("min_area", min_area_);
    this->get_parameter("max_area", max_area_);

    // パラメータが変更されたときに呼ばれるコールバック関数を登録
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ConeColorDetectorNode::parameterCallback, this,
                  std::placeholders::_1));

    // カメラ映像をサブスクライブ
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&ConeColorDetectorNode::imageCallback, this,
                  std::placeholders::_1));

    // 検出した色の中心座標(ピクセル)をPointCloud2としてパブリッシュ
    regions_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/color_regions", 10);

    // ★調整用★ マスク画像（白黒の画像）をパブリッシュするPublisher
    // image_transportだと圧縮トピック等が自動生成されてややこしいので、標準Publisherに変更
    mask_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/color_mask", 10);
  }

private:
  // パラメータがrqt_reconfigure等で変更されたときに呼ばれる関数
  rcl_interfaces::msg::SetParametersResult
  parameterCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
      if (param.get_name() == "h_min")
        h_min_ = param.as_int();
      if (param.get_name() == "h_max")
        h_max_ = param.as_int();
      if (param.get_name() == "h_min_2")
        h_min_2_ = param.as_int();
      if (param.get_name() == "h_max_2")
        h_max_2_ = param.as_int();
      if (param.get_name() == "s_min")
        s_min_ = param.as_int();
      if (param.get_name() == "s_max")
        s_max_ = param.as_int();
      if (param.get_name() == "v_min")
        v_min_ = param.as_int();
      if (param.get_name() == "v_max")
        v_max_ = param.as_int();
      if (param.get_name() == "min_area")
        min_area_ = param.as_int();
      if (param.get_name() == "max_area")
        max_area_ = param.as_int();
    }
    return result;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // 範囲1のマスク作成
    cv::Scalar hsv_min_1 = cv::Scalar(h_min_, s_min_, v_min_);
    cv::Scalar hsv_max_1 = cv::Scalar(h_max_, s_max_, v_max_);
    cv::Mat mask1;
    cv::inRange(hsv_image, hsv_min_1, hsv_max_1, mask1);

    // 範囲2のマスク作成
    cv::Scalar hsv_min_2 = cv::Scalar(h_min_2_, s_min_, v_min_);
    cv::Scalar hsv_max_2 = cv::Scalar(h_max_2_, s_max_, v_max_);
    cv::Mat mask2;
    cv::inRange(hsv_image, hsv_min_2, hsv_max_2, mask2);

    // 2つのマスクを結合
    cv::Mat mask;
    cv::bitwise_or(mask1, mask2, mask);

    // （オプション）ノイズ除去
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // ★調整用★ マスク画像をパブリッシュ
    // (cv::Mat -> sensor_msgs::msg::Image)
    sensor_msgs::msg::Image::SharedPtr mask_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
    mask_msg->header = msg->header;
    mask_publisher_->publish(*mask_msg); // ポインタの参照外しが必要

    // --- (これ以降は重心計算なので変更なし) ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    pcl::PointCloud<pcl::PointXYZ> centers_cloud;
    for (const auto &contour : contours) {
      double area = cv::contourArea(contour);
      // 面積が範囲外ならスキップ（小さすぎるノイズや大きすぎる背景を除外）
      if (area < min_area_ || area > max_area_)
        continue;

      // バウンディングボックスを取得
      cv::Rect bbox = cv::boundingRect(contour);

      cv::Moments m = cv::moments(contour);
      if (m.m00 > 0) {
        pcl::PointXYZ center_point;
        center_point.x = m.m10 / m.m00; // ピクセルX座標（重心）
        center_point.y = m.m01 / m.m00; // ピクセルY座標（重心）
        // z座標にバウンディングボックスの半幅（判定に使用）を格納
        // 幅と高さの大きい方の半分を使用
        center_point.z = std::max(bbox.width, bbox.height) / 2.0;
        centers_cloud.points.push_back(center_point);
      }
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(centers_cloud, output_msg);
    output_msg.header = msg->header;
    regions_publisher_->publish(output_msg);
  }

  // クラスメンバ変数の宣言
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      regions_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_publisher_; // ★調整用★ 標準Publisherに変更
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;

  // HSVしきい値の変数
  int h_min_, s_min_, v_min_, h_max_, s_max_, v_max_;
  int h_min_2_, h_max_2_;
  // 面積フィルタ
  int min_area_, max_area_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeColorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
