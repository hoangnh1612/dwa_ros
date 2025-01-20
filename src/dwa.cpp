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
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&DWANode::laser_callback, this, std::placeholders::_1));
    }
    double robot_radius = 0.05;
    double predict_time = 2.0;
    obstacles obs;
    Pose2D robot_pose;
    Pose2D goall;
    double dt = 0.1;
    double max_linear_vel = 0.5;
    double min_linear_vel = 0.0;
    double max_angular_vel = 0.5;
    double min_angular_vel = -0.5; 
    double resolution = 20;
    double w_obs = 1.0;
    double w_goal = 10.0;
    double w_velocity = 0.1;

    double max_acc_linear = 1.5;
    double min_acc_linear = -3.5;
    double max_acc_angular = 1.5;
    double min_acc_angular = -3.5;
    DynamicWindow calculate_window(Twist prev_vel)
    {
        DynamicWindow dw;
        std::cout<<robot_vel.linear<<std::endl;
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
bool check_collide(const Pose2D &robot_pose)
{
    for (auto ob : obs)
    {
        double dist = std::sqrt(
            std::pow(ob.position.x - robot_pose.position.x, 2) +
            std::pow(ob.position.y - robot_pose.position.y, 2)
        );
        if (dist < robot_radius)
        {
            std::cout << "COLLIDE" << std::endl;
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
        for(double i = 0; i < predict_time; i+= dt)
        {
            Pose2D pose;
            pose.position.x = robot_pose.position.x + v.linear * std::cos(robot_pose.theta) * i;
            pose.position.y = robot_pose.position.y + v.linear * std::sin(robot_pose.theta) * i;
            pose.theta = robot_pose.theta + v.angular * i;
            path.push_back(pose);
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
        double cost = 0.0;
        for(auto pose:p)
        {
            if (check_collide(pose))
            {
                cost += 10000;
                break;
            }
            else{
                cost += 1/calculate_min_distance(pose);
            }
        }
        return cost;
    }

    double calculate_goal_cost(Pose2D goal, Pose2D robot_pose)
    {
        return std::sqrt(std::pow(goal.position.x - robot_pose.position.x, 2) + 
                std::pow(goal.position.y - robot_pose.position.y, 2));
    }

    double calculate_heading_cost(PredictPath p)
    {
        double cost = 0.0;
        for (auto pose:p)
        {
            float dx = goall.position.x - pose.position.x;
            float dy = goall.position.y - pose.position.y;
            cost += std::abs(pose.theta - atan2(dy,dx));
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
        double o_cost, g_cost, v_cost, h_cost;
        double min_cost = 1000;
        Twist best_vel;
        for (auto v: vel)
        {
            PredictPath p = predict_trajectory(v);
            double obstacle_cost = calculate_obstacle_cost(p);
            double goal_cost = calculate_goal_cost(goal, p.back());
            double velocity_cost = calculate_velocity_cost(v, prev_vel);
            double heading_cost = calculate_heading_cost(p);
            double final_cost = w_obs * obstacle_cost + w_goal * goal_cost +velocity_cost * w_velocity + 10*heading_cost*w_goal;
            if (final_cost < min_cost)
            {
                min_cost = final_cost;
                best_vel = v;
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
            // std::cout<<"here"<<std::endl;
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
        // std::cout<<obs[3].position.x<<std::endl;
    }
    void visualise()
    {
        cv::Mat map = cv::Mat::zeros(1000, 1000, CV_8UC3);
        for (auto ob: obs)
        {
            Point2DPixel p = convertMeterToPixel(ob.position, 0.0, 0.0, 100);
            cv::circle(map, cv::Point(p.x, p.y), 1, cv::Scalar(255, 255, 255), -1);
        }
        Point2DPixel robot_p = convertMeterToPixel(robot_pose.position, 0.0, 0.0, 100);
        cv::circle(map, cv::Point(robot_p.x, robot_p.y), 5, cv::Scalar(0, 255, 0), -1);
        cv::imshow("Map", map);
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
        goall.position.x = 10;
        goall.position.y = 10;
        goall.theta = 0;
        robot_pose.position.x = msg->pose.pose.position.x;
        robot_pose.position.y = msg->pose.pose.position.y;
        robot_pose.theta = quat2theta(msg->pose.pose.orientation);

        DynamicWindow dw = calculate_window(prev_vel);
        ReachableVelocity vel = calculate_reachable_velocity(dw);
        std::cout<<"Num Obstacles: "<<obs.size()<<std::endl;
        std::cout<<"Min Linear: "<<dw.linear_min_vel<<" Max Linear: "<<dw.linear_max_vel<<" Min Angular: "<<dw.angular_min_vel<<" Max Angular: "<<dw.angular_max_vel<<std::endl;
        Twist robot_vel = DWA(vel, goall);

        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = robot_vel.linear;
        cmd_msg.angular.z = robot_vel.angular;
        cmd_pub_ ->publish(cmd_msg);
        prev_vel = robot_vel;
        visualise();
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    size_t count_;

  private:
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DWANode>());
  rclcpp::shutdown();
  return 0;
}