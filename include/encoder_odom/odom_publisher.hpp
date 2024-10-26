#include "encoder_odom/odom_publisher.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

OdomPublisher::OdomPublisher()
    : Node("encoder_odom"),
      last_time_(this->now()) {
    
    // pigpioの初期化
    pi_ = pigpio_start(NULL, NULL);
    if (pi_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio");
        return;
    }
    
    // エンコーダーの初期化 (変更後のピン番号を使用)
    left_encoder_ = std::make_unique<EncoderReader>(pi_, 17, 27, ticks_per_revolution_);  // GPIO17,27を左エンコーダに使用
    right_encoder_ = std::make_unique<EncoderReader>(pi_, 16, 26, ticks_per_revolution_); // GPIO16,26を右エンコーダに使用
    
    // パブリッシャーとブロードキャスターの設定
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // タイマーの設定（50Hzで更新）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&OdomPublisher::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Encoder odometry node initialized");
}

OdomPublisher::~OdomPublisher() {
    if (pi_ >= 0) {
        pigpio_stop(pi_);
    }
}

void OdomPublisher::updateOdometry() {
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    
    // エンコーダカウントの取得と角度計算
    static int prev_left_count = 0;
    static int prev_right_count = 0;
    int left_count = left_encoder_->getCount();
    int right_count = right_encoder_->getCount();
    
    double left_diff = (left_count - prev_left_count) * 2.0 * M_PI / ticks_per_revolution_;
    double right_diff = (right_count - prev_right_count) * 2.0 * M_PI / ticks_per_revolution_;
    
    prev_left_count = left_count;
    prev_right_count = right_count;
    
    // 車輪の移動距離計算
    double left_distance = left_diff * wheel_radius_;
    double right_distance = right_diff * wheel_radius_;
    
    // ロボットの移動距離と回転角計算
    double linear = (right_distance + left_distance) / 2.0;
    double angular = (right_distance - left_distance) / wheel_separation_;
    
    // 位置と向きの更新
    theta_ += angular;
    x_ += linear * cos(theta_);
    y_ += linear * sin(theta_);
}

void OdomPublisher::timerCallback() {
    updateOdometry();
    
    // オドメトリメッセージの作成
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // 位置と向きの設定
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    // TF2ブロードキャストメッセージの作成
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = odom_msg.header;
    transform_stamped.child_frame_id = odom_msg.child_frame_id;
    transform_stamped.transform.translation.x = x_;
    transform_stamped.transform.translation.y = y_;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = odom_msg.pose.pose.orientation;
    
    // メッセージのパブリッシュ
    odom_pub_->publish(odom_msg);
    tf_broadcaster_->sendTransform(transform_stamped);
}