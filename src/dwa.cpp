#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include<cmath>
#include<iostream>

#include "rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "obstacles.h"
#include"nav_msgs/msg/path.hpp"


using namespace std::chrono_literals;

Twist robot_vel;
std::vector<Pose2D> predict_path;
Twist prev_vel;
class DWANode : public rclcpp::Node
{
  public:
    DWANode()
    : Node("dwa"), count_(0)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DWANode::odom_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&DWANode::run, this));
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DWANode::laser_callback, this, std::placeholders::_1));
        path_pubs_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    }
    double robot_radius = 0.13;
    double predict_time = 2.0;
    obstacles obs;
    Pose2D robot_pose;
    Pose2D goall;
    double dt = 0.1;
    double max_linear_vel = 0.5;
    double min_linear_vel = 0.0;
    double max_angular_vel = 1.5;
    double min_angular_vel = -1.5; 
    double resolution = 20;
    double w_obs = 2.0;
    double w_goal = 2.0;
    double w_heading = 3.0;
    double w_velocity = 0.3;

    double max_acc_linear = 1.5;
    double min_acc_linear = -3.5;
    double max_acc_angular = 5.5;
    double min_acc_angular = -5.5;
    size_t best_trajectory_index;
    std::vector<PredictPath> path_container;
    DynamicWindow calculate_window(Twist prev_vel)
    {
        DynamicWindow dw;
        // std::cout<<robot_vel.linear<<std::endl;
        dw.linear_min_vel = std::max(min_linear_vel, prev_vel.linear - max_acc_linear * dt);
        dw.linear_max_vel = std::min(max_linear_vel, prev_vel.linear + max_acc_linear * dt);
        dw.angular_min_vel = std::max(min_angular_vel, prev_vel.angular - max_acc_angular * dt);
        dw.angular_max_vel = std::min(max_angular_vel, prev_vel.angular + max_acc_angular * dt);
        return dw;
    }
    double quat2theta(const geometry_msgs::msg::Quaternion q) const
    {
        double roll, pitch, yaw;
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    double normalize_angle(double angle)
    {
        return atan2(sin(angle), cos(angle));
    }
    bool check_collide(const Pose2D robot_pose)
    {
        for (auto ob : obs)
        {
            double dist = std::sqrt(
                std::pow(ob.position.x - robot_pose.position.x, 2) +
                std::pow(ob.position.y - robot_pose.position.y, 2)
            );
            if (dist < robot_radius)
            {
                return true;
            }
        }
        return false;
    }
    double calculate_min_distance(const Pose2D robot_pose)
    {
        double min_dist = 1000;
        for (auto ob : obs)
        {
            double dist = std::sqrt(std::pow(ob.position.x - robot_pose.position.x, 2) + 
                std::pow(ob.position.y - robot_pose.position.y, 2));
            if (dist < min_dist)
            {
                min_dist = dist;
            }
        }
        return min_dist;
    }
    PredictPath predict_trajectory(Twist v)
    {
        PredictPath path;
        Pose2D current_pose = robot_pose;  
        path.push_back(current_pose);      
        
        for(double i = dt; i < predict_time; i += dt)
        {
            Pose2D next_pose;
            
            next_pose.theta = normalize_angle(current_pose.theta + v.angular * dt);
            
            double avg_theta = current_pose.theta + (v.angular * dt) / 2.0;
            next_pose.position.x = current_pose.position.x + v.linear * std::cos(avg_theta) * dt;
            next_pose.position.y = current_pose.position.y + v.linear * std::sin(avg_theta) * dt;
            
            path.push_back(next_pose);
            current_pose = next_pose; 
        }
        return path;
    }
    ReachableVelocity calculate_reachable_velocity(DynamicWindow dw)
    {
        ReachableVelocity vel;
        double lin_resolution = (dw.linear_max_vel - dw.linear_min_vel) / 20;
        double ang_resolution = (dw.angular_max_vel - dw.angular_min_vel) / 20;
        for (double i = dw.linear_min_vel; i< dw.linear_max_vel; i+= lin_resolution)
        {
            for (double j = dw.angular_min_vel; j< dw.angular_max_vel; j+= ang_resolution)
            {
                Twist v;
                v.linear = i;
                v.angular = j;
                vel.push_back(v);
            }
        }
        return vel;
    }
    double calculate_obstacle_cost(PredictPath p)
    {
        double min_dist = 1000;
        for(auto pose:p)
        {
            if (check_collide(pose))
            {
                return 1000;
            }
            else{

                if (min_dist> calculate_min_distance(pose))
                {
                    min_dist = calculate_min_distance(pose);
                }
            }
        }
        return std::exp(-min_dist / robot_radius);
    }

    double calculate_goal_cost(Pose2D goal, PredictPath p)
    {
        Pose2D final_pose = p.back();
        for (auto pose:p)
        {
            if (std::sqrt(std::pow(goal.position.x - pose.position.x, 2) + 
                std::pow(goal.position.y - pose.position.y, 2)) < 0.25)
            {
                return 0;
            }
        }
        return std::sqrt(std::pow(goal.position.x - final_pose.position.x, 2) + 
                std::pow(goal.position.y - final_pose.position.y, 2));
    }
    double calculate_heading_cost(PredictPath p)
    {
        double cost = 0.0;
        for (auto pose:p)
        {
            float dx = goall.position.x - pose.position.x;
            float dy = goall.position.y - pose.position.y;
            float distance = hypot((robot_pose.position.x - goall.position.x), (robot_pose.position.y - goall.position.y));
            if (distance < 0.25)
            {
                cost += std::abs(normalize_angle(goall.theta - pose.theta));
                std::cout<<"GR"<<std::endl;
            }
            else
                cost += std::abs(normalize_angle(pose.theta - atan2(dy,dx)));
        }
        return cost;
    }

    double calculate_velocity_cost(Twist v, Twist prev_v)
    {
        return std::sqrt(std::pow(v.linear - prev_v.linear, 2) + 
                std::pow(v.angular - prev_v.angular, 2));
    }
    Twist DWA(ReachableVelocity vel, Pose2D goal)
    {
        path_container.clear();
        double o_cost, g_cost, v_cost, h_cost, vg_cost, gh_cost;
        double min_cost = 1000;
        Twist best_vel;
        for (size_t i = 0; i< vel.size(); i++)
        {
            Twist v = vel[i];
            PredictPath p = predict_trajectory(v);
            path_container.push_back(p);
            double obstacle_cost = calculate_obstacle_cost(p);
            double goal_cost = calculate_goal_cost(goal, p);
            
            double velocity_cost = calculate_velocity_cost(v, prev_vel);
            double heading_cost = calculate_heading_cost(p);
            double final_cost = w_obs * obstacle_cost + w_goal * goal_cost +velocity_cost * w_velocity + heading_cost*w_goal;
            if (final_cost < min_cost)
            {
                min_cost = final_cost;
                best_vel = v;
                best_trajectory_index = i;
                o_cost = obstacle_cost;
                g_cost = goal_cost;
                v_cost = velocity_cost;
                h_cost = heading_cost;
            }
        }
        std::cout<<"Best Velocity: "<<best_vel.linear<<" "<<best_vel.angular<<std::endl;
        std::cout<<"O: "<<o_cost<<" G: "<<g_cost<<" V: "<<v_cost<<" H: "<<h_cost<<std::endl;
        return best_vel;
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        obs.clear();
        Point2D obstacle_point;
        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;
        std::cout<<msg->angle_increment<<std::endl;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            if (!std::isnan(range) && std::isfinite(range))
            {            
                float angle = angle_min + i * angle_increment;
                
                float local_x = range * std::cos(angle);
                float local_y = range * std::sin(angle);
                float global_x = robot_pose.position.x + 
                            (local_x * std::cos(robot_pose.theta) - 
                                local_y * std::sin(robot_pose.theta));
                float global_y = robot_pose.position.y + 
                            (local_x * std::sin(robot_pose.theta) + 
                                local_y * std::cos(robot_pose.theta));
                
                obstacle_point.x = global_x;
                obstacle_point.y = global_y;
                obs.push_back(obstacle_point);
            }
        }
    }
    void visualise() {
        cv::Mat map = cv::Mat::zeros(1000, 1000, CV_8UC3);
        for (auto ob: obs) {
            Point2DPixel p = convertMeterToPixel(ob.position, 0.0, 0.0, 100);
            cv::circle(map, cv::Point(p.x, p.y), 1, cv::Scalar(255, 255, 255), -1);
        }

        Point2DPixel robot_p = convertMeterToPixel(robot_pose.position, 0.0, 0.0, 100);
        cv::circle(map, cv::Point(robot_p.x, robot_p.y), 5, cv::Scalar(0, 255, 0), -1);
        cv::Point arrow_end(
            robot_p.x + 15 * cos(robot_pose.theta),
            robot_p.y + 15 * sin(robot_pose.theta)
        );
        cv::arrowedLine(map, cv::Point(robot_p.x, robot_p.y), arrow_end, 
                        cv::Scalar(0, 255, 0), 2);


        for (size_t i = 0; i < path_container.size(); i++) {

            cv::Scalar path_color(0, 0, 255);  
            if (i == best_trajectory_index) {   
                path_color = cv::Scalar(255, 0, 0);  
            }
            
            auto& path = path_container[i];
            std::vector<cv::Point> trajectory_points;
            for (size_t j = 0; j < path.size(); j++) {
                Point2DPixel p = convertMeterToPixel(path[j].position, 0.0, 0.0, 100);
                trajectory_points.push_back(cv::Point(p.x, p.y));
                

                if (j % 5 == 0) {  
                    cv::Point dir_end(
                        p.x + 5 * cos(path[j].theta),
                        p.y + 5 * sin(path[j].theta)
                    );
                    cv::line(map, cv::Point(p.x, p.y), dir_end, path_color, 1);
                }
            }
            
            for (size_t j = 0; j < trajectory_points.size() - 1; j++) {
                cv::line(map, trajectory_points[j], trajectory_points[j + 1], 
                        path_color, 1);
            }
        }

    Point2DPixel goal_p = convertMeterToPixel(goall.position, 0.0, 0.0, 100);
    cv::circle(map, cv::Point(goal_p.x, goal_p.y), 8, cv::Scalar(255, 0, 255), -1);
    
    std::string info = "v=" + std::to_string(prev_vel.linear).substr(0,4) + 
                      " w=" + std::to_string(prev_vel.angular).substr(0,4);
    cv::putText(map, info, cv::Point(100, 300), cv::FONT_HERSHEY_SIMPLEX, 
                0.6, cv::Scalar(255,255,255), 2);

    cv::imshow("DWA Navigation", map);
    cv::waitKey(1);
    }
    Point2DPixel convertMeterToPixel(const Point2D point, const double origin_x, const double origin_y, const int resolution)
    {
        Point2DPixel p;
        p.x = 500+(int)((point.x - origin_x) * resolution);
        p.y = 500+(int)((point.y - origin_y) * resolution);
        return p;
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        goall.position.x = 2;
        goall.position.y = 2;
        goall.theta = 2;
        robot_pose.position.x = msg->pose.pose.position.x;
        robot_pose.position.y = msg->pose.pose.position.y;
        robot_pose.theta = quat2theta(msg->pose.pose.orientation);
    }

    void run()
    {
        DynamicWindow dw = calculate_window(prev_vel);
        ReachableVelocity vel = calculate_reachable_velocity(dw);
        std::cout<<"Min Linear: "<<dw.linear_min_vel<<" Max Linear: "<<dw.linear_max_vel<<" Min Angular: "<<dw.angular_min_vel<<" Max Angular: "<<dw.angular_max_vel<<std::endl;
        Twist robot_vel = DWA(vel, goall);
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = robot_vel.linear;
        cmd_msg.angular.z = robot_vel.angular;
        cmd_pub_ ->publish(cmd_msg);
        prev_vel = robot_vel;
        // std::cout<<"Pose: "<<robot_pose.position.x<<" "<<robot_pose.position.y<<" Orientation: "<<robot_pose.theta<<std::endl;
        visualise();
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pubs_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
  private:
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // execute run function
  rclcpp::spin(std::make_shared<DWANode>());
  rclcpp::shutdown();
  return 0;
}

