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
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <map>
#include <cmath> 
#include <vector>
#include <std_msgs/Empty.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>


// ros::Publisher marker_pub; // パブリッシャーをグローバルで宣言

class WaypointPublisher
{
public:
    WaypointPublisher(){
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
        main_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("main_waypoint", 1);
        goal_reached_pub_ = nh_.advertise<std_msgs::Bool>("goal_reached", 1);
        
        waypoint_num_sub_ = nh_.subscribe("waypoint_num", 1000, &WaypointPublisher::waypointNumCallback, this);
        goal_reached_sub_ = nh_.subscribe("goal_reached", 1000, &WaypointPublisher::goalReachedCallback, this);//これは使ってなさそう

        timer_callback_ = nh_.createTimer(ros::Duration(1.0), &WaypointPublisher::timerCallback, this);
        
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_odom_x_ = 0.0;
        robot_odom_y_ = 0.0;
        robot_r_.x = 0.0;
        robot_r_.y = 0.0;
        robot_r_.z = 0.0;
        robot_r_.w = 1.0;

        waypoint_num_ = 0;

        readWaypointsFromCSV(csv_file);
        }

private:
    ros::NodeHandle nh_;
    // ros::NodeHandle pnh_("~");
    // pnh_.getParam("current_location", current_location_);

    ros::Subscriber waypoint_sub_, goal_reached_sub_, waypoint_num_sub_;
    ros::Publisher marker_pub_, main_waypoint_pub_, goal_reached_pub_;
    ros::Timer timer_callback_;
    ros::Time timer_start_;
    ros::Time timer_now_;
    std_msgs::Int16 waypoint_num_msg_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    geometry_msgs::Point p_;
    geometry_msgs::Quaternion robot_r_;
    sensor_msgs::LaserScan::ConstPtr scan_;
    geometry_msgs::PoseStamped goal_; // 目標地点

    double roll_, pitch_, yaw_;
    double theta_;
    double angle_rad_, angle_deg_;
    double distance_, distance_judge_;
    double robot_odom_x_, robot_odom_y_;
    double robot_x_, robot_y_;
    bool goal_reached_;
    int waypoint_num_;

    // std::string csv_file = "/home/yamaguchi-a/catkin_ws/src/waypoint_publisher/csv/waypoint_odom.csv";
    std::string csv_file = "/home/yamaguchi-a/catkin_ws/src/waypoint_publisher/csv/waypoints.csv";

    
    void publishWaypoint();
    void publishWaypointsMarker();
    void timerCallback(const ros::TimerEvent&);
    void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
    void waypointNumCallback(const std_msgs::Int16::ConstPtr& msg);
    bool readWaypointsFromCSV(std::string csv_file);
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg);

};

    void WaypointPublisher::publishWaypointsMarker() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoints";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        // マーカーの寿命を設定（0にすると消えません）
        marker.lifetime = ros::Duration(0);

    

        for (const auto& waypoint : waypoints_) {
            geometry_msgs::Point p;
            p.x = waypoint.pose.position.x;
            p.y = waypoint.pose.position.y;
            p.z = waypoint.pose.position.z;
            marker.points.push_back(p);
        }

        marker_pub_.publish(marker);
    }

    void WaypointPublisher::timerCallback(const ros::TimerEvent& event) {
        publishWaypoint();
        publishWaypointsMarker();
            // std::cout << "goal_reached" << goal_reached_ << std::endl;

    }

    void WaypointPublisher::publishWaypoint(){//ここでwaypointの番号を増やしてpublishしている
        std::cout << "aaaaa" << std::endl;
        if (waypoint_num_ < waypoints_.size()) {//最後のwaypointに到着後の動きを司る？？？

            // if (goal_reached_) {
            //     goal_reached_ = false;
            //     waypoint_num_+= 1;
            // }
            std::cout << "goal_reached" << goal_reached_ << std::endl;

            if (goal_reached_ == 1) {
                std::cout << "goal_reached" << goal_reached_ << std::endl;
                std::cout << "waypoint_num_" << waypoint_num_ << std::endl;

                // goal_reached_ = false;
                // waypoint_num_ += 1;

                // ウェイポイントのパブリッシュ
                geometry_msgs::PoseStamped current_goal = waypoints_[waypoint_num_];
                main_waypoint_pub_.publish(current_goal);
                // goal_reached_ = false;
            }
            // std::cout << "goal_reached" << goal_reached_ << std::endl;

//この下があるとwaypointを常に垂れ流している？？？
            // std_msgs::Bool goal_reached_msg;
            // goal_reached_msg.data = goal_reached_;
            // goal_reached_pub_.publish(goal_reached_msg);
            // std::cout << "wayponint_index_:" << waypoint_num_ << std::endl;

            // geometry_msgs::PoseStamped current_goal = waypoints_[waypoint_num_];

            // // 現在のゴールを次のウェイポイントとしてpublish
            // main_waypoint_pub_.publish(current_goal);

            // std::cout << "goal_reached_:" << goal_reached_ << std::endl;
        }
    }

    bool WaypointPublisher::readWaypointsFromCSV(std::string csv_file){
        std::ifstream file(csv_file); //csvファイルを開く
        if(!file){ //ファイルが開けなかった場合
            ROS_ERROR("Cannot open file: %s", csv_file.c_str());
            return false;
        }

        std::string line; //1行ずつ読み込むための変数

        // std::getline(file, line); //最初の一行を読み飛ばす

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

    void WaypointPublisher::geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
    {
        tf::Quaternion quat;
        quaternionMsgToTF(geometry_quat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    }

    void WaypointPublisher::waypointNumCallback(const std_msgs::Int16::ConstPtr& msg) {
        waypoint_num_ = msg->data;
    }

    // void WaypointPublisher::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    //     double goal_.pose.position.x = msg->pose.position.x;
    //     double goal_.pose.position.y = msg->pose.position.y;
    //     double goal_.pose.position.x = msg->pose.position.z;
    // }

    void WaypointPublisher::goalReachedCallback(const std_msgs::Bool::ConstPtr& msg) {
        goal_reached_ = msg->data; // サブスクライブした値でgoal_reached_を更新
        // ROS_INFO("goal_reached_ = %s", goal_reached_ ? "true" : "false");
    }

int main(int argc, char **argv) {
    ros::init(argc, argv, "collision_monitor");

    WaypointPublisher cm;

    ros::Rate loop_rate(100);

    while(ros::ok()) {
        ros::spinOnce();
        // std::cout << "sub_waypoint_flag_:" << sub_waypoint_flag_ << std::endl;
        // std::cout << "state:" << state_ << std::endl;
        // twist_pub.publish(twist);
        loop_rate.sleep();
    }
    return 0;
}