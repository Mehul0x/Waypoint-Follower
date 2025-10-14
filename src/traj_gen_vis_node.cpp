#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "waypoint_follower/spline.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"




//using spline to generate a smooth trajectory
//publish path using nav_msgs/msg/Path
//publish waypoints using visualization_msgs/msg/Marker
//a downside is that there is no shared_ptr for the spline


class trajGen : public rclcpp::Node {
  public:
    trajGen() : Node("traj_gen_vis_node")
    {
        // Declare and load waypoint parameters from a YAML file
        this->declare_parameter<std::vector<double>>("waypoints.x", std::vector<double>());
        this->declare_parameter<std::vector<double>>("waypoints.y", std::vector<double>());
        this->declare_parameter<std::vector<double>>("waypoints.z", std::vector<double>()); // Declare Z parameter

        // Retrieve the parameters
        x = this->get_parameter("waypoints.x").as_double_array();
        y = this->get_parameter("waypoints.y").as_double_array();
        z = this->get_parameter("waypoints.z").as_double_array(); // Retrieve Z parameter

        // Check if waypoints were loaded successfully
        if (x.empty() || y.empty() || x.size() != y.size()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load valid waypoints. Ensure 'waypoints.x' and 'waypoints.y' parameters are set and have the same number of elements.");
            rclcpp::shutdown();
            return;
        }

        // Handle the Z coordinate: default to 0 if not provided or handle size mismatch
        if (z.empty()) {
            RCLCPP_INFO(this->get_logger(), "'waypoints.z' not provided. Defaulting all z-coordinates to 0.");
            z.resize(x.size(), 0.0); // Fill with zeros
        }

        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints (x, y, z).", x.size());
        
        // Keep the original structure by copying to waypt_ variables for the publish_waypoints function
        waypt_x = x;
        waypt_y = y;
        waypt_z = z; // Copy z coordinates

        // Create publishers
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("smoothed_path", 10);
        waypoint_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_markers", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&trajGen::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&trajGen::timer_callback, this));
    }

  private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        curr_x = current_pose_.position.x;
        curr_y = current_pose_.position.y;
        heading = tf2::getYaw(current_pose_.orientation);
    }

    void timer_callback()
    {
        std::vector<std::pair<double, double>> smoothed_path_points;
        // RCLCPP_INFO(this->get_logger(), "in callback");

        while(true){ //for path starting from bot
          
          if(x[0]==curr_x) break;
          if(x[0]<curr_x){ 
            x.erase(x.begin());
            y.erase(y.begin());
            continue;
          }
          x.insert(x.begin(), curr_x);
          y.insert(y.begin(), curr_y);
          
          break;
        }

        if(x.size()>2)
          spline.set_points(x,y, tk::spline::cspline);
        // RCLCPP_INFO(this->get_logger(), "spline generated");
        

        // Simulate a smoothed path by adding a few points in between
        for (size_t i = 0; i < x.size(); ++i) {
            double x_hat=x[i];
            double y_hat=y[i];

            for (int j = 0; j < 1000; ++j) {
                // double t = static_cast<double>(j) / 1000.0;

                //new x = speed * cos(heading) * t
                x_hat = 0.001 * speed * cos(std::atan( spline.deriv(1, x_hat))) + x_hat; //forward euler integration
                y_hat = spline(x_hat);

                if(x_hat<x[i+1])
                  smoothed_path_points.push_back({x_hat, y_hat});
                else{
                  smoothed_path_points.push_back({x[i], y[i]});
                  continue;
                }
            }
        }

        publish_waypoints();
        publish_path(smoothed_path_points);
    }

    void publish_waypoints()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        // Use waypt_x.size() for the loop condition as all vectors are guaranteed to have the same size
        for (size_t i=0; i < waypt_x.size(); i++)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom"; // IMPORTANT: Use a common frame
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "waypoints";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = waypt_x[i];
            marker.pose.position.y = waypt_y[i];
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1.0; // No rotation
            marker.scale.x = 0.3; // Diameter of the sphere
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.a = 1.0; // Must be 1.0 for the color to show
            marker.color.r = 1.0; // Red
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.lifetime = rclcpp::Duration(0, 0); // Lasts forever

            marker_array.markers.push_back(marker);
        }
        waypoint_marker_publisher_->publish(marker_array);
        // RCLCPP_INFO(this->get_logger(), "Published %zu waypoint markers.", marker_array.markers.size());
    }

    void publish_path(const std::vector<std::pair<double, double>>& path_points)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "odom"; // IMPORTANT: Must match the markers' frame
        path_msg.header.stamp = this->get_clock()->now();

        uint64_t time = 0;
        const uint64_t NS_PER_SEC = 1000000000UL;

        for (const auto& pt : path_points)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            time+=10e6;
            pose.header.stamp.sec = (int) (time / NS_PER_SEC);
            pose.header.stamp.nanosec = (uint32_t) (time % NS_PER_SEC);
            pose.pose.position.x = pt.first;
            pose.pose.position.y = pt.second;
            pose.pose.position.z = 0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        path_publisher_->publish(path_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose current_pose_;
    double curr_x =0, curr_y=0;

    double speed = 1; //dummy value
    double heading =0; 
    
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z; // New member for z coordinates
    std::vector<double> waypt_x;
    std::vector<double> waypt_y;
    std::vector<double> waypt_z; // New member for z waypoint markers
    tk::spline spline; 
    
};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trajGen>());
  rclcpp::shutdown();

  return 0;
}