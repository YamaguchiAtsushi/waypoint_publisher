#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>  // Boolメッセージ型を使用
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

class WaypointPublisher {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher twist_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher waypoint_pub_;
    ros::Subscriber goal_reached_sub_; // goal_reachedをサブスクライブするためのSubscriber

    geometry_msgs::Twist twist_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    int waypoint_index_;
    bool goal_reached_;
    double robot_x_, robot_y_, yaw_, odom_robot_x_, odom_robot_y_;

public:
    WaypointPublisher() : waypoint_index_(0), goal_reached_(false) {
        odom_sub_ = nh_.subscribe("odom", 1000, &WaypointPublisher::odomCallback, this);
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("next_waypoint", 1);

        // goal_reached_をサブスクライブ
        goal_reached_sub_ = nh_.subscribe("goal_reached", 1000, &WaypointPublisher::goalReachedCallback, this);
    }

    // goal_reachedを受け取るコールバック関数
    void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg) {
        goal_reached_ = msg->data; // サブスクライブした値でgoal_reached_を更新
        ROS_INFO("goal_reached_ = %s", goal_reached_ ? "true" : "false");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_robot_x_ = msg->pose.pose.position.x;
        odom_robot_y_ = msg->pose.pose.position.y;
    }

    bool readWaypointsFromCSV(const std::string& csv_file) {
        std::ifstream file(csv_file);
        if (!file) {
            ROS_ERROR("ファイルを開けません: %s", csv_file.c_str());
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string token;
            geometry_msgs::PoseStamped waypoint;

            std::getline(ss, token, ',');
            waypoint.pose.position.x = std::stod(token);

            std::getline(ss, token, ',');
            waypoint.pose.position.y = std::stod(token);

            std::getline(ss, token, ',');
            waypoint.pose.position.z = std::stod(token);

            waypoint.pose.orientation.w = 1.0;
            waypoints_.push_back(waypoint);
            ROS_INFO("ウェイポイント読み込み: x=%f, y=%f, z=%f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
        }
        file.close();
        return true;
    }

    void publishWaypointsMarker() {
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
        marker.color.a = 1.0;

        for (auto& waypoint : waypoints_) {
            geometry_msgs::Point p;
            p.x = waypoint.pose.position.x;
            p.y = waypoint.pose.position.y;
            p.z = waypoint.pose.position.z;
            marker.points.push_back(p);
        }

        marker_pub_.publish(marker);
    }

    void update() {
        if (waypoint_index_ < waypoints_.size()) {
            geometry_msgs::PoseStamped current_goal = waypoints_[waypoint_index_];

            // 現在のゴールを次のウェイポイントとしてpublish
            waypoint_pub_.publish(current_goal);

            if (goal_reached_) {
                goal_reached_ = false;
                waypoint_index_++;
            }
        }

        twist_pub_.publish(twist_);
        publishWaypointsMarker();
    }

    void run() {
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            update();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_follower");

    WaypointPublisher wp;
    std::string csv_file = "/path/to/your/waypoints.csv";

    if (!wp.readWaypointsFromCSV(csv_file)) {
        return 1;
    }

    wp.run();
    return 0;
}
