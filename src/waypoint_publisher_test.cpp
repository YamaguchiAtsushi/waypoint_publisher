#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <map>
#include <cmath> 
#include <vector>
#include <std_msgs/Empty.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>
#include <std_msgs/Bool.h>


#define GO_POSITION 0
#define PUBLISH_NEXT_MAIN_WAYPOINT 1
#define PUBLISH_NOW_MAIN_WAYPOINT 2
#define PUBLISH_SUB_WAYPOINT 3

int state_ = PUBLISH_NEXT_MAIN_WAYPOINT;
float robot_odom_x_, robot_odom_y_;
float robot_x_, robot_y_;
geometry_msgs::Quaternion robot_r_;

geometry_msgs::Twist twist; // 指令する速度、角速度
geometry_msgs::PoseStamped goal; // 目標地点
sensor_msgs::LaserScan::ConstPtr scan;
std_msgs::Float32 robot_speed_msg;

// ros::Publisher marker_pub; // パブリッシャーをグローバルで宣言

double roll, pitch, yaw;
bool timer_flag = false;

class WaypointPublisher
{
public:
    WaypointPublisher() : sub_waypoint_flag_(0), waypoint_index_(0), goal_reached_(false){
        robot_speed_pub_ = nh_.advertise<std_msgs::Float32>("speed", 1000, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("waypoint", 1);
        goal_reached_pub_ = nh_.advertise<std_msgs::Bool>("goal_reached", 10);
        
        odom_sub_ = nh_.subscribe("ypspur_ros/odom", 1000, &WaypointPublisher::odomCallback, this);
        amcl_sub_ = nh_.subscribe("/amcl_pose", 1000, &WaypointPublisher::amclPoseCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 10, &WaypointPublisher::scanCallback, this);
        goal_reached_sub_ = nh_.subscribe("goal_reached", 1000, &WaypointPublisher::goalReachedCallback, this);
        obstacle_edge_point_sub_ = nh_.subscribe("obstacle_edge_point", 1000, &WaypointPublisher::obstacleEdgePointCallback, this);

        timer_callback_ = nh_.createTimer(ros::Duration(1.0), &WaypointPublisher::timerCallback, this);    
        std::string csv_file = "/home/yamaguchi-a/catkin_ws/src/waypoint_publisher/csv/waypoint_odom.csv";
        readWaypointsFromCSV(csv_file);
        }
    //ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {//near_waypointなどはこっちでやる
        robot_odom_x_ = msg->pose.pose.position.x;
        robot_odom_y_ = msg->pose.pose.position.y;
        // std::cout << "robot_odom_x_:" << robot_odom_x_ << "robot_odom_y_;" << robot_odom_y_ << std::endl;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_) {
        scan = scan_;
        to_sub_waypoint();//ここからーーーーーーーーーーーーーーーーーーーーー
        // detect_obstacle();
    }

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_r_ = msg->pose.pose.orientation;
        // ROS_INFO("Current estimated pose: [robot_x_: %f, robot_y_: %f, theta: %f]", robot_x_, robot_y_, tf::getYaw(msg->pose.pose.orientation));
    }

    void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg) {
        goal_reached_ = msg->data; // サブスクライブした値でgoal_reached_を更新
        // ROS_INFO("goal_reached_ = %s", goal_reached_ ? "true" : "false");
    }

    void obstacleEdgePointCallback(const std_msgs::Float32::ConstPtr& msg) {//障害物回避のwaypointを決める
        obstacle_edge_point_ = msg->data;
        // ROS_INFO("Obstacle edge point distance = %f", distance);
    }

    void timerCallback(const ros::TimerEvent&) {
        switch (state_) {
            case GO_POSITION:
                break;

            case PUBLISH_NEXT_MAIN_WAYPOINT:
                publishWaypoint();

                break;

            case PUBLISH_NOW_MAIN_WAYPOINT:
                std::cout << "PUBLISH_NOW_MAIN_WAYPOINT" << std::endl;
                break;

            case PUBLISH_SUB_WAYPOINT:
                std::cout << "PUBLISH_SUB_WAYPOINT" << std::endl;
                publishSubWaypoint();
                break;
        }
        // ROS_INFO("Timer callback!!!!!");
    }


private:
    ros::NodeHandle nh_;

    ros::Subscriber odom_sub_, scan_sub_, amcl_sub_, goal_reached_sub_, obstacle_edge_point_sub_;
    ros::Publisher cmd_pub_, twist_pub_, marker_pub_, waypoint_pub_, goal_reached_pub_, robot_speed_pub_;

    ros::Timer timer_callback_;
    ros::Time timer_start;
    ros::Time timer_now;

    std::vector<geometry_msgs::PoseStamped> waypoints_;


    int waypoint_index_;
    bool goal_reached_;
    int sub_waypoint_flag_;
    float obstacle_edge_point_;

    // std::string csv_file = "/home/yamaguchi-a/catkin_ws/src/waypoint_publisher/csv/waypoint_odom.csv";

    void publishSubWaypoint(){
        if (waypoint_index_ < waypoints_.size()) {
            std::cout << "goal_reached" << goal_reached_ << std::endl;

            if (goal_reached_) {
                std::cout << "goal_reached" << goal_reached_ << std::endl;

                // goal_reached_ = false;
                waypoint_index_ += 1;

                // ウェイポイントのパブリッシュ
                geometry_msgs::PoseStamped current_goal = waypoints_[waypoint_index_];
                waypoint_pub_.publish(current_goal);
                goal_reached_ = false;

            }

            // std::cout << "goal_reached" << goal_reached_ << std::endl;

            std_msgs::Bool goal_reached_msg;
            goal_reached_msg.data = goal_reached_;
            goal_reached_pub_.publish(goal_reached_msg);
            std::cout << "wayponint_index_:" << waypoint_index_ << std::endl;

            geometry_msgs::PoseStamped current_goal = waypoints_[waypoint_index_];

            // 現在のゴールを次のウェイポイントとしてpublish
            waypoint_pub_.publish(current_goal);

            // std::cout << "goal_reached_:" << goal_reached_ << std::endl;
        }
    }

    bool readWaypointsFromCSV(std::string csv_file){
        std::ifstream file(csv_file); //csvファイルを開く
        if(!file){ //ファイルが開けなかった場合
            ROS_ERROR("Cannot open file: %s", csv_file.c_str());
            return false;
        }

        std::string line; //1行ずつ読み込むための変数

        std::getline(file, line); //最初の一行を読み飛ばす

        while (std::getline(file, line)) {
                std::istringstream ss(line);//文字列をカンマで区切るための変数
                std::string token;//1つのデータを格納する変数
                geometry_msgs::PoseStamped waypoint; //waypointを格納する変数

                // x座標の読み込み
                std::getline(ss, token, ',');//カンマで区切った文字列を1つずつ読み込む
                waypoint.pose.position.x = std::stod(token);//文字列をdouble型に変換してwaypointに格納

                // y座標の読み込み
                std::getline(ss, token, ',');
                waypoint.pose.position.y = std::stod(token);

                // z座標の読み込み
                std::getline(ss, token, ',');
                waypoint.pose.position.z = std::stod(token);

                // orientationを初期化
                waypoint.pose.orientation.x = 1.0;

                // ベクトルに追加
                waypoints_.push_back(waypoint);
                
                // デバッグメッセージを追加
                ROS_INFO("Read waypoint: x=%f, y=%f, z=%f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
            }
        file.close();
        return true;
        }

    void publishWaypoint(){
        if (waypoint_index_ < waypoints_.size()) {
            // if (goal_reached_) {
            //     goal_reached_ = false;
            //     waypoint_index_+= 1;
            // }
            std::cout << "goal_reached" << goal_reached_ << std::endl;

            if (goal_reached_) {
                std::cout << "goal_reached" << goal_reached_ << std::endl;

                // goal_reached_ = false;
                waypoint_index_ += 1;

                // ウェイポイントのパブリッシュ
                geometry_msgs::PoseStamped current_goal = waypoints_[waypoint_index_];
                waypoint_pub_.publish(current_goal);
                goal_reached_ = false;

            }
            // std::cout << "goal_reached" << goal_reached_ << std::endl;

            std_msgs::Bool goal_reached_msg;
            goal_reached_msg.data = goal_reached_;
            goal_reached_pub_.publish(goal_reached_msg);
            std::cout << "wayponint_index_:" << waypoint_index_ << std::endl;

            geometry_msgs::PoseStamped current_goal = waypoints_[waypoint_index_];

            // 現在のゴールを次のウェイポイントとしてpublish
            waypoint_pub_.publish(current_goal);

            // std::cout << "goal_reached_:" << goal_reached_ << std::endl;
        }
    }


    void to_sub_waypoint(){
        // std::cout << "sub_waypoint_flag_:" << sub_waypoint_flag_ << std::endl;

        if(sub_waypoint_flag_ == 1){
            near_sub_waypoint();
        }
        else{
            near_now_main_waypoint();
        }
    }

    void near_now_main_waypoint(){
        if(false){
            state_ = PUBLISH_NEXT_MAIN_WAYPOINT;
        }
        else{
            detect_obstacle();
            std::cout << "----------------------" << std::endl;
        }
    }

    void near_sub_waypoint(){
        if(true){
            state_ = PUBLISH_NOW_MAIN_WAYPOINT;
        }
        else{
            passed_line();
        }

    }

    void passed_line(){
        if(true){
            state_ = PUBLISH_NEXT_MAIN_WAYPOINT;
        }
        else{
            detect_obstacle();
        }

    }

    void detect_obstacle(){//calcAngleをつかって目的地との間にある障害物を検知するようにする！！！！！！！！！！！！！！！！！！！！！
        robot_speed_msg.data = 1.0;

        std::cout << "detect_obstacle called" << std::endl;
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float angle_rad = scan->angle_min + i * scan->angle_increment;
            float angle_deg = angle_rad * (180.0 / M_PI); // ラジアンを度数法に変換
            float distance = scan->ranges[i];
            float distance_judge_ = 100;

            // std::cout << "Obstacle found: back, distance_judge = " << distance_judge_ << std::endl;
            geometry_msgs::Point p;
            p.x = distance * std::cos(angle_rad); // X座標
            p.y = distance * std::sin(angle_rad); // Y座標
            p.z = 0; // Z座標は0に設定

            distance_judge_ = scan->ranges[i] * std::cos(angle_rad); // std::cosの入力はrad

            if (-10 < angle_deg && angle_deg < 10) {
                // distance_judge_ = scan->ranges[i] * std::cos(angle_rad); // std::cosの入力はrad
                if ((-1.5 < distance_judge_ && distance_judge_ < -0.5) || (0.5 < distance_judge_ && distance_judge_ < 1.5)) {
                    std::cout << "1_0.5~1.5" << std::endl;
                    // geometry_msgs::Point p;
                    // p.x = distance * std::cos(angle_rad); // X座標
                    // p.y = distance * std::sin(angle_rad); // Y座標
                    // p.z = 0; // Z座標は0に設定
                    // marker.points.push_back(p); // ポイントをマーカーに追加
                    // robot_speed = 0.5;
                    robot_speed_msg.data = 0.3;

                    timer_flag = false;
                }else if(-0.5 <= distance_judge_ && distance_judge_ <= 0.5){
                    std::cout << " 1_~0.5" <<  std::endl;
                    // geometry_msgs::Point p;
                    // p.x = distance * std::cos(angle_rad); // X座標
                    // p.y = distance * std::sin(angle_rad); // Y座標
                    // p.z = 0; // Z座標は0に設定
                    // marker.points.push_back(p); // ポイントをマーカーに追加
                    // robot_speed = 0.0;
                    robot_speed_msg.data = 0.0;

                    timer();//detect_obstacleがずっと呼ばれていると抜け出せない

                }else{
                    continue; //startに戻る
                    timer_flag = false;
                }
            }
            
            // else if ((angle_deg >= -90 && angle_deg <= -10) || (angle_deg >= 10 && angle_deg <= 90)) {
            //     // distance_judge = scan_->ranges[i] * std::sin(angle_rad); // std::sinの入力はrad
            //     if ((-1.5 < distance_judge_ && distance_judge_ < -0.5) || (0.5 < distance_judge_ && distance_judge_ < 1.5)) {
            //         std::cout << "2_0.5~1.5" << std::endl;
            //         // std::cout << "Obstacle found: side, distance_judge = " << distance_judge << std::endl;
            //         // geometry_msgs::Point p;
            //         // p.x = distance * std::cos(angle_rad); // X座標
            //         // p.y = distance * std::sin(angle_rad); // Y座標
            //         // p.z = 0; // Z座標は0に設定
            //         // robot_speed = 0.5;   
            //         robot_speed_msg.data = 0.5;

            //         timer_flag = false;
            //     }
            //     else if(-0.5 <= distance_judge_ && distance_judge_ <= 0.5){
            //         std::cout << " 2_~0.5" <<  std::endl;
            //         // std::cout << "Obstacle found: back, distance_judge_ = " << distance_judge_ << std::endl;
            //         // geometry_msgs::Point p;
            //         // p.x = distance * std::cos(angle_rad); // X座標
            //         // p.y = distance * std::sin(angle_rad); // Y座標
            //         // p.z = 0; // Z座標は0に設定
            //         // marker.points.push_back(p); // ポイントをマーカーに追加
            //         // robot_speed = 0.0;
            //         robot_speed_msg.data = 0.0;

            //         timer();

            //     }else{
            //         continue; //startに戻る
            //         timer_flag = false;
            //     }
            // }
        }
        robot_speed_pub_.publish(robot_speed_msg);
    }


    void stop(){

    }

    void timer(){
        if(timer_flag == false){
            timer_start = ros::Time::now();
            timer_flag = true;
        }
        timer_now = ros::Time::now();
        if((timer_now - timer_start).toSec() > 3.0){
            obstacle_near_now_main_waypoint();
        }

        ROS_INFO("timer_now - timer_start = %f", (timer_now - timer_start).toSec());
        // else{
        //     detect_obstacle();
        // }
    }

    void obstacle_near_now_main_waypoint(){
        if(false){
            state_ = PUBLISH_SUB_WAYPOINT;
        }
        else{
            state_ = PUBLISH_NEXT_MAIN_WAYPOINT;
        }

    }

    double calcAngle(geometry_msgs::PoseStamped goal){
        double theta = atan2(goal.pose.position.y - robot_odom_y_, goal.pose.position.x - robot_odom_x_);
        while (theta <= -M_PI || M_PI <= theta)
        {
            if (theta <= -M_PI)
                theta = theta + 2 * M_PI;
            else
                theta = theta - 2 * M_PI;
        }

        geometry_quat_to_rpy(roll, pitch, yaw, robot_r_);

        while (yaw <= -M_PI || M_PI <= yaw)
        {
            if (yaw <= -M_PI)
                yaw = yaw + 2 * M_PI;
            else
                yaw = yaw - 2 * M_PI;
        }

        theta = theta - yaw;
        
        return theta;
    }

    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
    {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    }

    int near_position(geometry_msgs::PoseStamped goal)
    {
        double difx = robot_odom_x_ - goal.pose.position.x;
        double dify = robot_odom_y_ - goal.pose.position.y;
        return (sqrt(difx * difx + dify * dify) < 0.2);
    }

    void go_position(geometry_msgs::PoseStamped goal)
    {
        double k_v = 3.0;
        double k_w = 1.6;

        double v = 1.0;
        double w = 0.0;

        double theta = atan2(goal.pose.position.y - robot_odom_y_, goal.pose.position.x - robot_odom_x_);
        while (theta <= -M_PI || M_PI <= theta)
        {
            if (theta <= -M_PI)
                theta = theta + 2 * M_PI;
            else
                theta = theta - 2 * M_PI;
        }

        geometry_quat_to_rpy(roll, pitch, yaw, robot_r_);

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
            v = k_v * ((goal.pose.position.x - robot_odom_x_) * (goal.pose.position.x - robot_odom_x_) + (goal.pose.position.y - robot_odom_y_) * (goal.pose.position.y - robot_odom_y_));
        else
            v = -k_v * ((goal.pose.position.x - robot_odom_x_) * (goal.pose.position.x - robot_odom_x_) + (goal.pose.position.y - robot_odom_y_) * (goal.pose.position.y - robot_odom_y_));

        twist.linear.x = 1.0;
        twist.angular.z = w;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_publisher_test");
    // ros::NodeHandle nh;

    WaypointPublisher wp;


    ros::Rate loop_rate(100);
    robot_x_ = 0.0;
    robot_y_ = 0.0;
    robot_odom_x_ = 0.0;
    robot_odom_y_ = 0.0;
    robot_r_.x = 0.0;
    robot_r_.y = 0.0;
    robot_r_.z = 0.0;
    robot_r_.w = 1.0;

    while(ros::ok()) {
        ros::spinOnce();
        // std::cout << "sub_waypoint_flag_:" << sub_waypoint_flag_ << std::endl;
        // std::cout << "state:" << state_ << std::endl;
        // twist_pub.publish(twist);
        loop_rate.sleep();
    }
    return 0;
}