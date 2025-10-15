#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <iostream>

class TrackingController : public rclcpp::Node
{
public:
    TrackingController() : Node("tracking_controller_node")
    {

        // Subscriptions
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/smoothed_path", 10, std::bind(&TrackingController::path_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrackingController::odom_callback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&TrackingController::scan_callback, this, std::placeholders::_1));

        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10); //need twiststamped
        
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  //20Hz
            std::bind(&TrackingController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Tracking controller node has been started.");
    }


private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received an empty path. Stopping the robot.");
            current_path_ = nullptr;
            stop_robot();
        } else {
            current_path_ = msg;
            // RCLCPP_INFO(this->get_logger(), "Received a new path with %zu poses.", msg->poses.size());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        odometry_received_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;
        scan_received_ = true;
    }

    void control_loop()
    {
        if (!odometry_received_ || !scan_received_ || current_path_->poses.empty())
        {
            // If no path or odometry, do nothing (or publish zero velocity)
            return;
        }

        geometry_msgs::msg::Point current_position = current_pose_.position;
        double current_yaw = tf2::getYaw(current_pose_.orientation); //should publish this

        // Find the closest point on the path to the robot
        size_t closest_point_idx = 0;
        double min_dist_sq = -1.0;

        // for (size_t i = 0; i < current_path_->poses.size(); ++i)
        // {
        //     double dist_sq = pow(current_position.x - current_path_->poses[i].pose.position.x, 2) +
        //                      pow(current_position.y - current_path_->poses[i].pose.position.y, 2);
        //     if (min_dist_sq < 0 || dist_sq < min_dist_sq)
        //     {
        //         min_dist_sq = dist_sq;
        //         closest_point_idx = i;
        //     }
        // }

        // Search forward from the closest point to find the lookahead point
        size_t target_idx = closest_point_idx;
        while (target_idx < current_path_->poses.size() - 1)
        {
            double lookahead_dist_sq = pow(current_position.x - current_path_->poses[target_idx].pose.position.x, 2) +
                                       pow(current_position.y - current_path_->poses[target_idx].pose.position.y, 2);
            if (lookahead_dist_sq >= pow(lookahead_distance, 2))
            {
                break;
            }
            target_idx++;
        }

        geometry_msgs::msg::Point target_point = current_path_->poses[target_idx].pose.position;

        // 2. Calculate the required angular velocity
        double alpha = atan2(target_point.y - current_position.y, target_point.x - current_position.x) - current_yaw;
        double dist_to_target = std::sqrt(pow(target_point.x - current_position.x, 2) + pow(target_point.y - current_position.y, 2));
        double angular_velocity = (2.0 * linear_velocity * sin(alpha)) / dist_to_target;


        // 3. Publish the Twist message
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        
        // Check if we are at the end of the path
        double dist_to_final_point = std::sqrt(pow(current_path_->poses.back().pose.position.x - current_position.x, 2) + 
                                               pow(current_path_->poses.back().pose.position.y - current_position.y, 2));

        auto [path_valid, obstacle_angle] = check_path_validity();

        if (dist_to_final_point < 0.2) // Stop if within 20cm of the final goal
        {
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "Reached the end of the path. Robot stopped.");
            current_path_ = nullptr; // Clear path to prevent restarting
        }
        else if(path_valid)
        {
            twist_msg.twist.linear.x = linear_velocity;
            twist_msg.twist.angular.z = angular_velocity;
        }
        else{
            twist_msg.twist.linear.x = linear_velocity;

            if(obstacle_angle <= 5.39 && obstacle_angle >= 4.02) // obstacle on right
                twist_msg.twist.angular.z = 0.0; //go straight
            else 
                twist_msg.twist.angular.z = 0.15; //turn left
        }

        cmd_vel_pub_->publish(twist_msg);
    }
    
    void stop_robot() {
        RCLCPP_INFO(this->get_logger(), "Robot stopped");
        auto stop_msg = geometry_msgs::msg::TwistStamped();
        stop_msg.twist.linear.x = 0.0;
        stop_msg.twist.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_msg);
    }

    std::pair<bool, double> check_path_validity() {
        if(!last_scan_) {
            return {true,0}; // Assume path is valid if no scan data
        }
        auto angle_increment = last_scan_->angle_increment;
        auto angle_min = last_scan_->angle_min;

        for(uint32_t i=0; i<last_scan_->ranges.size(); ++i) {
            float dist = last_scan_->ranges[i];
            auto angle = angle_min + i * angle_increment;

            if(dist < 0.7 && dist >last_scan_->range_min ){
                if(angle > 5.39 || angle < 0.88){
                    RCLCPP_INFO(this->get_logger(), "Obstacle detected at angle: %.2f radians, distance: %.2f meters", angle, dist);
                    linear_velocity = 0.1;
                    return {false, angle};
                }
            }
     
        }
        linear_velocity = 1.0;
        return {true,0};
    }

    // Member variables
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    geometry_msgs::msg::Pose current_pose_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    bool odometry_received_ = false;
    bool scan_received_ = false; 

    double lookahead_distance = 0.5;
    double linear_velocity = 1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingController>());
    rclcpp::shutdown();
    return 0;
}

