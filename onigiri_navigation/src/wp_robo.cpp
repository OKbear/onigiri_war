#include "wp_robo.h"

#define GAIN_CHASE -0.007
#define PNT_START_CHASE 9

MyPose way_point[] = {
    {1.700,  0.001,  0.000, true},
    {1.700, -0.636, -0.991, false},
    {1.921, -0.750, -0.774, false},
    {1.900, -0.757, -3.140, false},
    {1.640, -0.747, -3.140, true},
    {2.150, -0.650,  1.548, true},
    {1.965, -1.345,  0.027, false},
    {3.000, -1.115,  0.860, false},
    {3.345, -0.825,  0.387, false},
/*
    {2.479, -0.702, -0.023, true},
    {2.660, -0.721,  0.000, false},
    {2.399, -1.516,  0.800, false},
*/
/* ボツ
    {0.162,  0.112,  0.774, false},
    {0.838,  0.805,  0.819, false},
    {1.026,  0.794, -0.021, true},
    {0.773,  0.648, -0.794, false},
    {0.943, -0.005, -0.989, false},
    {1.320,  0.012,  0.005, false},
    {1.541,  0.000, -0.052, true},
    {1.331, -0.038, -2.385, false},
    {0.706, -0.440, -1.740, false},
    {0.850, -0.797, -0.813, false},
    {1.017, -0.778, -0.004, true},
    {0.950, -0.009,  0.583, false},
    {1.266,  0.040,  0.782, false},
    {1.308, -0.040, -0.921, false},
    {1.985, -0.802, -0.708, false},
    {2.035, -0.776,  1.593, false},
    {2.033, -0.636, -2.949, false},
    {1.614, -0.719, -3.049, true},
    {1.977, -0.754,  0.006, false},
*/

    {999, 999, 999, false}};

RoboCtrl::RoboCtrl() : it_(node), ac("/move_base", true)
{
    ros::NodeHandle node;

    //購読するtopic名
    image_sub_ = it_.subscribe("camera/image_raw", 1, &RoboCtrl::imageCb, this);

    //配布するtopic名
    twist_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //内部関数初期化
    front_speed_ = 0.5;
    turn_speed_ = 0.0;
    diff_position_ = 0.0;

    time_idle = ros::Time::now();

    // まず最初はウェイポイント
    state_ = STATE_WAYPOINT;

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
    is_wp_sent_ = false;
    dest_point_ = 0;
}

void RoboCtrl::moveRobo()
{
    //速度データ型宣言
    geometry_msgs::Twist twist;

    checkWaypoint();

    // STATE_CHASEの場合 P制御で追いかける
    if (state_ == STATE_CHASE)
    {
        ros::Duration tick = ros::Time::now() - time_approach_run_;
        double tickdbl = tick.toSec();
        if (tickdbl <= 1.0)
        {
            front_speed_ = 0.0;
            turn_speed_ = diff_position_ * GAIN_CHASE;
            ROS_INFO("CHASE PHASE(1) %f", tickdbl);
        }
        else if (tickdbl <= 2.0)
        {
            front_speed_ = 0.3;
            turn_speed_ = diff_position_ * GAIN_CHASE;
            ROS_INFO("CHASE PHASE(2) %f", tickdbl);
        }
        else if (tickdbl > 2.0)
        {
            front_speed_ = 0.7;
            turn_speed_ = diff_position_ * GAIN_CHASE;
            ROS_INFO("CHASE PHASE(3) %f", tickdbl);
        }
        ROS_INFO("みっけ〜〜〜♪");
    }

    /* ここは入らない */
    if (state_ == STATE_RETREAT)
    {
        ros::Duration tick = ros::Time::now() - time_retreat;
        double tickdbl = tick.toSec();
        double rand_spd = -(rand() % 3) / 10.0;
        static double sign = 1;
        sign *= -1;
        if (tickdbl <= 1.0)
        {
            front_speed_ = rand_spd;
            turn_speed_ = sign * rand_spd;
            ROS_INFO("RETREAT PHASE(1) %f", tickdbl);
        }
        else if (tickdbl > 1.0)
        {
            front_speed_ = 0.0;
            front_speed_ = 0.0;
            state_ = STATE_WAYPOINT;
        }
    }

    if (state_ == STATE_BACK)
    {
        ros::Duration tick = ros::Time::now() - time_back;
        double tickdbl = tick.toSec();
        if (tickdbl <= 2.0)
        {
            front_speed_ = -0.2;
            turn_speed_ = 0.0;
        }
        else if (tickdbl > 2.0)
        {
            front_speed_ = 0.0;
            front_speed_ = 0.0;
            state_ = STATE_WAYPOINT;
        }
            
    }
    // ウェイポイント終わったらSTATE_IDLEにしてその場で回る
    if (state_ == STATE_IDLE)
    {
        if( ros::Time::now() - time_idle > ros::Duration(2.0))
        {
            front_speed_= (rand() % 4 - 2) / 10.0;
            turn_speed_ = (rand() % 10 - 5) / 2.0;
            time_idle = ros::Time::now();
        }
    }

    ROS_INFO("Current State: %s", getStateByString(state_).c_str());
    ROS_INFO("Position Diff: %f", diff_position_);

    //ROS速度データに内部関数値を代入
    if (state_ != STATE_WAYPOINT)
    {
        twist.linear.x = front_speed_;
        twist.angular.z = turn_speed_;
        twist_pub_.publish(twist);
    }
}

void RoboCtrl::checkWaypoint()
{
    // ウェイポイント送信済みか？
    if (is_wp_sent_)
    {
        // ウェイポイント送信済みなら到着結果の確認をする
        // 追跡モード中はis_wp_sent_はfalseなので確認しない
        bool isSucceeded = ac.waitForResult(ros::Duration(0.5));
        // 結果を見て、成功ならSucceeded、失敗ならFailedと表示
        actionlib::SimpleClientGoalState state = ac.getState();

        // 到着済みもしくは到着タイムアウトか?
        if (isSucceeded)
        {
            is_wp_sent_ = false;
            ROS_INFO("WP Arrived: No.%d (%s)", dest_point_ + 1, state.toString().c_str());

            if(state == actionlib::SimpleClientGoalState::ABORTED)
            {
                
                ROS_INFO("NO MORE WAYPOINT!!!");
                state_ = STATE_IDLE;
            }
            else
            {
                time_back = ros::Time::now();
                ros::Duration(0.5).sleep();

                if( way_point[dest_point_].is_checkpoint )
                {
                    state_ = STATE_BACK;
                }
                ++dest_point_;
            }
        }
        else
        {
            // 到着してないなら何もしない
            ROS_INFO("WP Not Arrived: No.%d (%s)", dest_point_ + 1, state.toString().c_str());
        }
    }
    else
    {
        // ウェイポイント送信済みでない場合
        if (state_ == STATE_WAYPOINT)
        {
            sendWaypoint(dest_point_);
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
        state_ = STATE_IDLE;
        return;
    }

    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[n_ptr].yaw);

    ROS_INFO("Sending goal: No.%d", n_ptr + 1);
    ROS_INFO("X=%f Y=%f th=%f", way_point[n_ptr].x, way_point[n_ptr].y, way_point[n_ptr].yaw);

    // サーバーにgoalを送信
    ac.sendGoal(goal);

    is_wp_sent_ = true;
}

void RoboCtrl::cancelWaypoint()
{
    // ウェイポイントのキャンセル。STATE_CHASEへの移行時に呼ばれる
    ac.cancelAllGoals();
    ROS_INFO("WAYPOINT CANCELED. START CHASING!!");

    // STATE_WAYPOINT復帰時にウェイポイントを送り直すようにしておく
    is_wp_sent_ = false;
}

void RoboCtrl::odomCallback(const nav_msgs::Odometry &odom)
{
    return;
}

void RoboCtrl::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    const int centerpnt = 200;
    const int range = 10;
    static EState laststate = state_;

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
    ROS_INFO("AREA: %f", area);

    if ((x >= 0) && (x <= 400) && (area > 40000) && (dest_point_ >= PNT_START_CHASE) )
    {
        // 敵が見つかったら追跡する
        diff_position_ = x - centerpnt;

        // STATE_CHASEに入る前の状態を保存
        switch (state_)
        {
        /* ここは入らない */
        case STATE_WAYPOINT:
            cancelWaypoint();
            laststate = STATE_IDLE;
            break;
        
        case STATE_IDLE:
            laststate = STATE_IDLE;
            time_approach_run_ = ros::Time::now();
            break;

        /* ここは入らない */
        case STATE_RETREAT:
            laststate = STATE_IDLE;
            break;

        /* ここは入らない */
        case STATE_BACK:
            laststate = STATE_IDLE;
            break;
        

        default:
            break;
        }
        // STATE_CHASEに遷移
        state_ = STATE_CHASE;
    }
    else
    {
        // 敵を見失う or そもそも敵を見つけていない場合
        diff_position_ = 0;
        // STATE_CHASEに入る前の状態を戻す
        if (state_ == STATE_CHASE)
        {
            state_ = laststate;
        }
    }

    //ROS_INFO("obj x=%d y=%d",x,y);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, mask);
    cv::waitKey(3);
}

std::string RoboCtrl::getStateByString(EState state){
    switch( state ) {
        case STATE_IDLE:
            return "IDLE";
            break;
        case STATE_WAYPOINT:
            return "WAY POINT";
            break;
        case STATE_CHASE:
            return "CHASE";
            break;
        case STATE_RETREAT:
            return "RETREAT";
            break;
        case STATE_BACK:
            return "BACK";
            break;
        default:
            return "UNKNOWN STATE";
            break;
    }
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
