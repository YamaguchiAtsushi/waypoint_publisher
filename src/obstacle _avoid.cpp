#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <map>
#include <cmath> 
#include <vector>
#include <std_msgs/Empty.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>

#define GO_POSITION 0
#define HALF_SPEED 1
#define STOP 2
#define TIMER 3
#define PUBLISH_SUB_WAYPOINT 4
#define SKIP_WAYPOINT 5
#define MAX_SPEED 6


int state_ = GO_POSITION;

double robot_odom_x, robot_odom_y;
double robot_x, robot_y;


geometry_msgs::Quaternion robot_r;

geometry_msgs::Twist twist; // 指令する速度、角速度
geometry_msgs::PoseStamped goal; // 目標地点
geometry_msgs::Twist speed_msg;

ros::Publisher marker_pub; // パブリッシャーをグローバルで宣言

double roll, pitch, yaw;


void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    robot_odom_x = msg->pose.pose.position.x;
    robot_odom_y = msg->pose.pose.position.y;
    std::cout << "robot_odom_x:" << robot_odom_x << "robot_odom_y;" << robot_odom_y << std::endl;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser"; // RViz内のフレーム
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacle_points"; // 名前空間
    marker.action = visualization_msgs::Marker::ADD; // マーカーを追加
    marker.type = visualization_msgs::Marker::POINTS; // マーカータイプをポイントに設定
    marker.scale.x = 0.1; // 点のサイズ（X軸）
    marker.scale.y = 0.1; // 点のサイズ（Y軸）
    marker.color.r = 1.0f; // 赤色
    marker.color.g = 0.0f; // 緑色
    marker.color.b = 0.0f; // 青色
    marker.color.a = 1.0; // アルファ（透明度）

    for (size_t i = 0; i < scan_->ranges.size(); i++) {
        float angle_rad = scan_->angle_min + i * scan_->angle_increment;
        float angle_deg = angle_rad * (180.0 / M_PI); // ラジアンを度数法に変換
        float distance = scan_->ranges[i];
        float distance_judge_ = 100;



        if (-10 < angle_deg && angle_deg < 10) {
            distance_judge_ = scan_->ranges[i] * std::cos(angle_rad); // std::cosの入力はrad
            if ((-1.0 < distance_judge_ && distance_judge_ < -0.3) || (0.3 < distance_judge_ && distance_judge_ < 1.0)) {
                std::cout << "Obstacle found: back, distance_judge = " << distance_judge_ << std::endl;
                geometry_msgs::Point p;
                p.x = distance * std::cos(angle_rad); // X座標
                p.y = distance * std::sin(angle_rad); // Y座標
                p.z = 0; // Z座標は0に設定
                marker.points.push_back(p); // ポイントをマーカーに追加
                state_ = HALF_SPEED;
            }else if(-0.3 <= distance_judge_ && distance_judge_ <= 0.3){
                std::cout << "Obstacle found: back, distance_judge_ = " << distance_judge_ << std::endl;
                geometry_msgs::Point p;
                p.x = distance * std::cos(angle_rad); // X座標
                p.y = distance * std::sin(angle_rad); // Y座標
                p.z = 0; // Z座標は0に設定
                marker.points.push_back(p); // ポイントをマーカーに追加
                state_ = STOP;
            }else{
                state_ = GO_POSITION;
            }
        } 


        // else if ((angle_deg >= -90 && angle_deg <= -10) || (angle_deg >= 10 && angle_deg <= 90)) {
        //     distance_judge_ = scan_->ranges[i] * std::sin(angle_rad); // std::sinの入力はrad
        //     if (-0.5 < distance_judge_ && distance_judge_ < 0.5) {
        //         std::cout << "Obstacle found: side, distance_judge_ = " << distance_judge_ << std::endl;
        //         geometry_msgs::Point p;
        //         p.x = distance * std::cos(angle_rad); // X座標
        //         p.y = distance * std::sin(angle_rad); // Y座標
        //         p.z = 0; // Z座標は0に設定
        //         marker.points.push_back(p); // ポイントをマーカーに追加
        //     }
        // }

    }
    marker_pub.publish(marker); // マーカーを公開
}

void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

int near_position(geometry_msgs::PoseStamped goal)
{
    double difx = robot_odom_x - goal.pose.position.x;
    double dify = robot_odom_y - goal.pose.position.y;
    return (sqrt(difx * difx + dify * dify) < 0.2);
}

void go_position(geometry_msgs::PoseStamped goal)
{
    double k_v = 3.0;
    double k_w = 1.6;

    double v = 1.0;
    double w = 0.0;

    double theta = atan2(goal.pose.position.y - robot_y, goal.pose.position.x - robot_x);
    while (theta <= -M_PI || M_PI <= theta)
    {
        if (theta <= -M_PI)
            theta = theta + 2 * M_PI;
        else
            theta = theta - 2 * M_PI;
    }

    geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

    while (yaw <= -M_PI || M_PI <= yaw)
    {
        if (yaw <= -M_PI)
            yaw = yaw + 2 * M_PI;
        else
            yaw = yaw - 2 * M_PI;
    }

    theta = theta - yaw;

    while (theta <= -M_PI || M_PI <= theta)
    {
        if (theta <= -M_PI)
            theta = theta + 2 * M_PI;
        else
            theta = theta - 2 * M_PI;
    }

    w = k_w * theta;

    if (theta <= M_PI / 2 && theta >= -M_PI / 2)
        v = k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
    else
        v = -k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));

    twist.linear.x = 1.0;
    twist.angular.z = w;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_avoid");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 1000, odom_callback);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("speed", 1000)
    //ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Rate loop_rate(100);
    robot_x = 0.0;
    robot_y = 0.0;
    robot_r.x = 0.0;
    robot_r.y = 0.0;
    robot_r.z = 0.0;
    robot_r.w = 1.0;


    while(ros::ok()) {
        ros::spinOnce();
        std::cout << "state:" << state_ << std::endl;
        switch (state_) {
            case GO_POSITION:
                std::cout << "state:GO_POSITION" << std::endl;
                goal.pose.position.x = 3.0;
                goal.pose.position.y = 0.0;
                speed_msg.linear.x = 0.5;
                speed_msg.angular.z = 0.0;
                speed_pub.publish(msg);
                go_position(goal);
                if (near_position(goal))
                {
                    speed_msg.linear.x = 0.5;
                    speed_msg.angular.z = 0.0;
                    speed_pub.publish(msg);
                    state_ = STOP;
                }
                break;

            case HALF_SPEED:
                std::cout << "state:HALF_SPEED" << std::endl;
                goal.pose.position.x = 3.0;
                goal.pose.position.y = 0.0;
                twist.linear.x = 0.3;
			    twist.angular.z = 0.0;
                // go_position(goal);
                if (near_position(goal))
                {
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    state_ = STOP;
                }
                break;

            case STOP:
                std::cout << "state:STOP" << std::endl;
                goal.pose.position.x = 3.0;
                goal.pose.position.y = 0.0;
                twist.linear.x = 0.0;
			    twist.angular.z = 0.0;
                std::cout << "twist.linear.x" << twist.linear.x << "twist.angular.z" << twist.angular.z << std::endl;
                break;

            
            default:
                break;
    }

        twist_pub.publish(twist);
        loop_rate.sleep();
    }
    return 0;
}