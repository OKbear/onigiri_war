#ifndef WP_ROBO_H
#define WP_ROBO_H

#include "ros/ros.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
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
};

class RoboCtrl
{
protected:
    enum EState
    {
        STATE_IDLE = 0,
        STATE_WAYPOINT = 1,
        STATE_CHASE = 2,
        STATE_TELEOP = 3,
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

    EState m_state;

    // ロボット制御用
    double m_diffPos;
    double m_frontspeed;
    double m_turnspeed;
    ros::Time m_timechasestr;

    // ウェイポイント制御用
    int m_destPnt;
    bool m_isSent;
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
};

#endif