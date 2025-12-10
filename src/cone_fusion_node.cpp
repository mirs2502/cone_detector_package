#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 3D->2D投影 ライブラリ
#include <image_geometry/pinhole_camera_model.h>

// TF2 (座標変換) ライブラリ
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// メッセージ同期 ライブラリ (今回は使わない シンプル実装)
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>


class ConeFusionNode : public rclcpp::Node
{
public:
	ConeFusionNode()
        : Node("cone_fusion_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
   	 {
        // カメラ内部パラメータをサブスクライブ (通常のQoSに変更)
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", 10, // <-- 通常のQoS(10)に変更
            std::bind(&ConeFusionNode::cameraInfoCallback, this, std::placeholders::_1));

        // LiDARの候補点群をサブスクライブ
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cone_centers", 10,
            std::bind(&ConeFusionNode::lidarCallback, this, std::placeholders::_1));

        // カメラの色候補（ピクセル座標）をサブスクライブ
        color_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/color_regions", 10,
            std::bind(&ConeFusionNode::colorCallback, this, std::placeholders::_1));

        // 最終的に確定したコーンの位置をパブリッシュ
        confirmed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/confirmed_cones", 10);

        RCLCPP_INFO(this->get_logger(), "Fusion node started. Waiting for CameraInfo...");
    }

private:
    // カメラ情報が来たら、カメラモデルを初期化する
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!cam_model_.fromCameraInfo(msg)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera model");
        } else {
            cam_model_initialized_ = true;
            camera_frame_id_ = msg->header.frame_id; // "camera_link" など
            RCLCPP_INFO(this->get_logger(), "Camera model initialized!");
            // 一度受信したらサブスクライバを停止
            camera_info_sub_.reset();
        }
    }

    // カメラの色候補（ピクセル座標）を受信したら、最新のものを保持
    void colorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_color_regions_ = msg;
    }

    // LiDARの候補点群がメインのトリガー
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
    {
        // 必要な情報（カメラモデル、色候補）がまだ来ていないなら何もしない
        if (!cam_model_initialized_ || !latest_color_regions_)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for camera model or color regions...");
            return;
        }

        // LiDARの点群 (PCL形式)
        pcl::PointCloud<pcl::PointXYZ> lidar_cloud;
        pcl::fromROSMsg(*lidar_msg, lidar_cloud);

        // カメラの色候補 (PCL形式、x,yにピクセル座標が入っている)
        pcl::PointCloud<pcl::PointXYZ> color_cloud;
        pcl::fromROSMsg(*latest_color_regions_, color_cloud);

        // 確定したコーン（フュージョンが成功した点）を格納するPCL点群
        pcl::PointCloud<pcl::PointXYZ> confirmed_cloud;

        // TF取得: LiDARの座標系 (lidar_msg->header.frame_id) から
        //         カメラの座標系 (camera_frame_id_) への変換
        geometry_msgs::msg::TransformStamped tf_stamped;
        try
        {
            tf_stamped = tf_buffer_.lookupTransform(
                camera_frame_id_,        // 変換先のフレーム (例: "camera_link")
                lidar_msg->header.frame_id, // 変換元のフレーム (例: "laser")
                tf2::TimePointZero,    // 最新のTFを取得
                std::chrono::milliseconds(100)); // 待機時間
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                        lidar_msg->header.frame_id.c_str(), camera_frame_id_.c_str(), ex.what());
            return;
        }

        // LiDARの全候補点をループ処理
        for (const auto &lidar_point : lidar_cloud.points)
        {
            // 1. 3D座標をLiDAR座標系からカメラ座標系に変換
            geometry_msgs::msg::PointStamped point_in_lidar;
            point_in_lidar.header = lidar_msg->header;
            point_in_lidar.point.x = lidar_point.x;
            point_in_lidar.point.y = lidar_point.y;
            point_in_lidar.point.z = lidar_point.z;

            geometry_msgs::msg::PointStamped point_in_camera;
            tf2::doTransform(point_in_lidar, point_in_camera, tf_stamped);

            // 2. 3D座標（カメラ系）を2Dピクセル座標に投影
            cv::Point3d pt_3d(point_in_camera.point.x, point_in_camera.point.y, point_in_camera.point.z);
            cv::Point2d pt_2d; // 投影されたピクセル座標(x, y)が入る
            
            // カメラの前方にある点（Z>0）だけを投影
            if (pt_3d.z > 0)
            {
                pt_2d = cam_model_.project3dToPixel(pt_3d);
            } else {
                continue; // カメラの後ろにある点は無視
            }

            // 3. 投影されたピクセルが、色の候補と一致するかチェック
            bool found_color_match = false;
            
            // 全ての「色の候補点（ピクセル座標）」をチェック
            for (const auto &color_point : color_cloud.points)
            {
                // ピクセル距離を計算
                double distance = std::hypot(pt_2d.x - color_point.x, pt_2d.y - color_point.y);
                
                // 投影された点と色候補の距離が 20ピクセル以内 なら一致とみなす (★要調整★)
                if (distance < 100.0)
                {
                    found_color_match = true;
                    break; // 一致する色を一つ見つけたらループを抜ける
                }
            }

            // 4. 一致したら、「本物のコーン」として元のLiDARの点（map座標系など）を追加
            if (found_color_match)
            {
                // 元の座標（lidar_point）を追加
                confirmed_cloud.points.push_back(lidar_point);
            }
        }

        // 確定したコーンの点群をパブリッシュ
        if (!confirmed_cloud.points.empty())
        {
            sensor_msgs::msg::PointCloud2 confirmed_msg;
            pcl::toROSMsg(confirmed_cloud, confirmed_msg);
            confirmed_msg.header = lidar_msg->header; // 元のフレームIDと時刻
            confirmed_pub_->publish(confirmed_msg);
        }
    }

    // TF用
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // カメラモデル用
    image_geometry::PinholeCameraModel cam_model_;
    bool cam_model_initialized_ = false;
    std::string camera_frame_id_;

    // サブスクライバ
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr color_sub_;
    
    // パブリッシャ
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr confirmed_pub_;

    // 最新の色候補を保持する
    sensor_msgs::msg::PointCloud2::SharedPtr latest_color_regions_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeFusionNode>());
    rclcpp::shutdown();
    return 0;
}
