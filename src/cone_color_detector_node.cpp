#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.hpp> // マスク画像配信用に追加

// #defineでのハードコードを削除

class ConeColorDetectorNode : public rclcpp::Node
{
public:
    ConeColorDetectorNode() : Node("cone_color_detector_node")
    {
        // パラメータを宣言 (H: 0-179, S: 0-255, V: 0-255)
        // (デフォルト値はオレンジ色の例)
        this->declare_parameter("h_min", 0);
        this->declare_parameter("s_min", 130);
        this->declare_parameter("v_min", 50);
        this->declare_parameter("h_max", 25);
        this->declare_parameter("s_max", 255);
        this->declare_parameter("v_max", 230);

        // 宣言したパラメータを変数に読み込む
        this->get_parameter("h_min", h_min_);
        this->get_parameter("s_min", s_min_);
        this->get_parameter("v_min", v_min_);
        this->get_parameter("h_max", h_max_);
        this->get_parameter("s_max", s_max_);
        this->get_parameter("v_max", v_max_);

        // パラメータが変更されたときに呼ばれるコールバック関数を登録
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ConeColorDetectorNode::parameterCallback, this, std::placeholders::_1));

        // カメラ映像をサブスクライブ
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ConeColorDetectorNode::imageCallback, this, std::placeholders::_1));

        // 検出した色の中心座標(ピクセル)をPointCloud2としてパブリッシュ
        regions_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/color_regions", 10);

        // ★調整用★ マスク画像（白黒の画像）をパブリッシュするPublisher
        mask_publisher_ = image_transport::create_publisher(this, "/color_mask");
    }

private:
    // パラメータがrqt_reconfigure等で変更されたときに呼ばれる関数
    rcl_interfaces::msg::SetParametersResult parameterCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "h_min") h_min_ = param.as_int();
            if (param.get_name() == "s_min") s_min_ = param.as_int();
            if (param.get_name() == "v_min") v_min_ = param.as_int();
            if (param.get_name() == "h_max") h_max_ = param.as_int();
            if (param.get_name() == "s_max") s_max_ = param.as_int();
            if (param.get_name() == "v_max") v_max_ = param.as_int();
        }
        return result;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // パラメータからHSVの範囲をcv::Scalarとして作成
        cv::Scalar hsv_min = cv::Scalar(h_min_, s_min_, v_min_);
        cv::Scalar hsv_max = cv::Scalar(h_max_, s_max_, v_max_);

        cv::Mat mask;
        cv::inRange(hsv_image, hsv_min, hsv_max, mask);

        // （オプション）ノイズ除去
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // ★調整用★ マスク画像をパブリッシュ
        // (cv::Mat -> sensor_msgs::msg::Image)
        sensor_msgs::msg::Image::SharedPtr mask_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        mask_msg->header = msg->header;
        mask_publisher_.publish(mask_msg);

        // --- (これ以降は重心計算なので変更なし) ---
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        pcl::PointCloud<pcl::PointXYZ> centers_cloud;
        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < 100) continue;

            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0)
            {
                pcl::PointXYZ center_point;
                center_point.x = m.m10 / m.m00; // ピクセルX座標
                center_point.y = m.m01 / m.m00; // ピクセルY座標
                center_point.z = 0;
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr regions_publisher_;
    image_transport::Publisher mask_publisher_; // ★調整用★
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // HSVしきい値の変数
    int h_min_, s_min_, v_min_, h_max_, s_max_, v_max_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeColorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
