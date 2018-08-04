#include "wp_robo.h"

#define GAIN_CHASE -0.01
#define PNT_START_CHASE 16

MyPose way_point[] = {
    {0.858,  0.743,  0.005},
    {0.843,  0.095, -0.730},
    {1.506, -0.009, -0.021},
    {1.393,  0.329,  0.287},
    {2.035,  0.698,  0.005},
    {2.236,  0.742, -0.016},
    {2.386,  0.650, -0.545},
/*   {1.923, 1.082, -1.551},
    {1.923, 1.082, 0.039},
    {1.923, 1.082, -2.529},
    //{ 1.035,  0.233,  0.000},
    {1.035, 0.233, -0.720},
    {1.980, -0.604, 0.116},
    {1.980, -0.604, 1.624},
    {1.980, -0.604, -3.135},
    {1.980, -0.604, 0.740},
    {2.584, 0.417, -3.133},
    {2.584, 0.417, 2.300},
    {1.923, 1.082, -2.970},
    {1.923, 1.082, -1.551},
    {1.923, 1.082, 0.039},
    {1.923, 1.082, -2.529},
    {1.035, 0.233, 0.000},
    {1.035, 0.233, -0.720},
    {1.980, -0.604, 0.116},
    {1.980, -0.604, 1.624},
    {1.980, -0.604, -3.135},
    {1.980, -0.604, 0.116},*/
    {999, 999, 999}};

RoboCtrl::RoboCtrl() : it_(node), ac("move_base", true)
{
    ros::NodeHandle node;

    //購読するtopic名
    image_sub_ = it_.subscribe("/red_bot/camera/image_raw", 1, &RoboCtrl::imageCb, this);

    //配布するtopic名
    twist_pub_ = node.advertise<geometry_msgs::Twist>("/red_bot/cmd_vel", 1);

    //内部関数初期化
    m_frontspeed = 0.5;
    m_turnspeed = 0.0;
    m_diffPos = 0.0;

    // まず最初はウェイポイント
    m_state = STATE_WAYPOINT;

    initWaypoint();
    cv::namedWindow(OPENCV_WINDOW);
}

RoboCtrl::~RoboCtrl()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void RoboCtrl::initWaypoint()
{
    // アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。
    // この例では５秒間待つ(ブロックされる)
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("The server comes up");
    m_isSent = false;
    m_destPnt = 0;
}

void RoboCtrl::moveRobo()
{
    //速度データ型宣言
    geometry_msgs::Twist twist;

    checkWaypoint();

    // STATE_CHASEの場合 P制御で追いかける
    if (m_state == STATE_CHASE)
    {
        ros::Duration tick = ros::Time::now() - m_timechasestr;
        double tickdbl = tick.toSec();
        if (tickdbl <= 8.0)
        {
            m_frontspeed = 0.1;
            m_turnspeed = m_diffPos * GAIN_CHASE;
            ROS_INFO("CHASE PHASE(1) %f", tickdbl);
        }
        else if (tickdbl <= 16.0)
        {
            m_frontspeed = 0.0;
            m_turnspeed = m_diffPos * GAIN_CHASE;
            ROS_INFO("CHASE PHASE(2) %f", tickdbl);
        }
        else if (tickdbl > 16.0)
            m_frontspeed = 0.0;
        m_turnspeed = -m_diffPos * GAIN_CHASE;
        ROS_INFO("CHASE PHASE(3) %f", tickdbl);
    }

    // ウェイポイント終わったらSTATE_IDLEにしてその場で回る
    if (m_state == STATE_IDLE)
    {
        m_frontspeed = 0.0;
        m_turnspeed = 0.5;
    }

    ROS_INFO("NOW %d", m_state);
    ROS_INFO("diff %f", m_diffPos);

    //ROS速度データに内部関数値を代入
    if (m_state == STATE_CHASE || m_state == STATE_IDLE)
    {
        twist.linear.x = m_frontspeed;
        twist.angular.z = m_turnspeed;
        //twist_pub_.publish(twist);
    }
}

void RoboCtrl::checkWaypoint()
{
    // ウェイポイント送信済みか？
    if (m_isSent)
    {
        // ウェイポイント送信済みなら到着結果の確認をする
        // 追跡モード中はm_isSentはfalseなので確認しない
        bool isSucceeded = ac.waitForResult(ros::Duration(0.5));
        // 結果を見て、成功ならSucceeded、失敗ならFailedと表示
        actionlib::SimpleClientGoalState state = ac.getState();

        // 到着済みか?
        if (isSucceeded)
        {
            // 到着済みなら次の点を送信する
            m_isSent = false;
            ROS_INFO("WP Reached: No.%d (%s)", m_destPnt + 1, state.toString().c_str());
            ros::Duration(1.0).sleep();
            sendWaypoint(++m_destPnt);
        }
        else
        {
            // 到着してないなら何もしない
            ROS_INFO("WP not reached: No.%d (%s)", m_destPnt + 1, state.toString().c_str());
        }
    }
    else
    {
        // ウェイポイント送信済みでない場合
        if (m_state == STATE_WAYPOINT)
        {
            // STATE_WAYPOINTのときウェイポイント送信(初回時やSTATE_CHASEから復帰時)
            // STATE_CHASE, STATE_IDLEでは何もしない)
            sendWaypoint(m_destPnt);
        }
    }
}

void RoboCtrl::sendWaypoint(int n_ptr)
{
    // STATE_WAYPOINTの場合のみ呼ばれる
    // ウェイポイントの更新 or 初回時 or STATE_CHASEからの復帰
    move_base_msgs::MoveBaseGoal goal;
    // map(地図)座標系
    goal.target_pose.header.frame_id = "map";
    // 現在時刻
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = way_point[n_ptr].x;
    goal.target_pose.pose.position.y = way_point[n_ptr].y;

    // 最終ウェイポイントに達したらSTATE_IDLEに遷移して抜ける。
    if (goal.target_pose.pose.position.x == 999)
    {
        m_state = STATE_IDLE;
        return;
    }

    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[n_ptr].yaw);

    ROS_INFO("Sending goal: No.%d", n_ptr + 1);
    ROS_INFO("X=%f Y=%f th=%f", way_point[n_ptr].x, way_point[n_ptr].y, way_point[n_ptr].yaw);

    // サーバーにgoalを送信
    ac.sendGoal(goal);

    m_isSent = true;
}

void RoboCtrl::cancelWaypoint()
{
    // ウェイポイントのキャンセル。STATE_CHASEへの移行時に呼ばれる
    ac.cancelAllGoals();
    ROS_INFO("WAYPOINT CANCELED. START CHASING!!");

    // STATE_WAYPOINT復帰時にウェイポイントを送り直すようにしておく
    m_isSent = false;
}

void RoboCtrl::odomCallback(const nav_msgs::Odometry &odom)
{
    return;
}

void RoboCtrl::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    const int centerpnt = 320;
    const int range = 10;
    static EState laststate = m_state;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(60 - range, 100, 100), cv::Scalar(60 + range, 255, 255), mask); // 色検出でマスク画像の作成
    //cv::bitwise_and(cv_ptr->image,mask,image);

    cv::Moments mu = cv::moments(mask, false);
    cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

    double area = mu.m00;
    int x = mu.m10 / mu.m00;
    int y = mu.m01 / mu.m00;
    ROS_INFO("AREA = %f", area);

    if ((x >= 0) && (x <= 640) && (area > 500000) && (m_destPnt >= PNT_START_CHASE))
    {
        // 敵が見つかったら追跡する
        m_diffPos = x - centerpnt;

        // STATE_CHASEに入る前の状態を保存
        switch (m_state)
        {
        case STATE_WAYPOINT:
            cancelWaypoint();
        case STATE_IDLE:
            laststate = m_state;
            m_timechasestr = ros::Time::now();
            break;
        default:
            break;
        }
        // STATE_CHASEに遷移
        m_state = STATE_CHASE;
    }
    else
    {
        // 敵を見失う or そもそも敵を見つけていない場合
        m_diffPos = 0;
        // STATE_CHASEに入る前の状態を戻す
        if (m_state == STATE_CHASE)
        {
            m_state = laststate;
        }
    }

    //ROS_INFO("obj x=%d y=%d",x,y);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, mask);
    cv::waitKey(3);
}

int main(int argc, char **argv)
{
    //ROSのノード初期化
    ros::init(argc, argv, "robo_ctrl");
    RoboCtrl robo_ctrl;
    ros::Rate r(20);

    while (ros::ok())
    {
        robo_ctrl.moveRobo();
        ros::spinOnce();
        r.sleep();
    }
}
