#ifndef NO_WP_ROBO_H
#define NO_WP_ROBO_H

#include "ros/ros.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

// ウェイポイント用MoveBase型定義
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// OpenCV用の何か
static const std::string OPENCV_WINDOW = "Image window";

// ウェイポイント用構造体定義
struct MyPose
{
    double x;
    double y;
    double yaw;
    bool is_checkpoint;
};

class RoboCtrl
{
protected:
    enum EState
    {
        STATE_IDLE = 0,
        STATE_WAYPOINT = 1,
        STATE_CHASE = 2,
        STATE_RETREAT = 3,
        STATE_BACK = 4,
        STATE_KIMEUCHI = 5,
    };

    ros::NodeHandle node;
    ros::Subscriber odom_sub_;
    ros::Subscriber bumper_sub_;
    ros::Subscriber Laser_sub_;
    ros::Publisher twist_pub_;

    //ROS時間
    ros::Time push_time_;
    //ROS変化秒
    ros::Duration under_time_;

    cv::Mat hsv;
    cv::Mat mask;
    cv::Mat image;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    EState state_;

    // ロボット制御用
    double diff_position_;
    double front_speed_;
    double turn_speed_;
    ros::Time time_approach_run_;
    ros::Time time_retreat;
    ros::Time time_back;
    ros::Time time_idle;
    ros::Time time_kimeuchi;

    // ウェイポイント制御用
    int dest_point_;
    bool is_wp_sent_;
    MoveBaseClient ac;

  public:
    // コンストラクタ
    RoboCtrl();

    // デストラクタ
    ~RoboCtrl();

    // ウェイポイント初期化関数
    void initWaypoint();

    // ウェイポイント到着チェック関数
    void checkWaypoint();

    // ウェイポイント送信関数
    void sendWaypoint(int n_ptr);

    // ウェイポイントキャンセル関数
    void cancelWaypoint();

    // ロボット動作用メインループ関数
    void moveRobo();

    // オドメトリ受信時のコールバック関数
    void odomCallback(const nav_msgs::Odometry &odom);

    // OpenCVの画像受信コールバック関数
    void imageCb(const sensor_msgs::ImageConstPtr &msg);

    std::string getStateByString(EState state);
};

#endif