#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <utility>
#include <functional>
#include <chrono>

using std::placeholders::_1;

class PIDFollow : public rclcpp::Node
{
public:
    PIDFollow()
    : Node("pid_follow"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // ---------- PID parameters ----------
        d_ref_ = 0.7;
        Kp_ = 1.2;
        Ki_ = 0.0;
        Kd_ = 0.2;
        Kp_angle_ = 3.0;

        v_max_ = 0.6;
        w_max_ = 1.5;

        // ---------- DWA safety parameters ----------
        max_accel_ = 0.6;
        max_delta_w_ = 1.5;
        predict_time_ = 0.8;
        dt_sim_ = 0.1;
        v_reso_ = 0.03;
        w_reso_ = 0.10;
        robot_radius_ = 0.22;

        // PID memory
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_time_ = this->get_clock()->now();

        // ROS
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PIDFollow::scan_callback, this, _1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PIDFollow::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "PID follower with DWA obstacle layer started");
    }

private:
    // ---------- Parameters ----------
    double d_ref_;
    double Kp_;
    double Ki_;
    double Kd_;
    double Kp_angle_;

    double v_max_;
    double w_max_;

    double max_accel_;
    double max_delta_w_;
    double predict_time_;
    double dt_sim_;
    double v_reso_;
    double w_reso_;
    double robot_radius_;

    // ---------- PID memory ----------
    double integral_;
    double prev_error_;
    rclcpp::Time prev_time_;

    // ---------- ROS ----------
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // --------------------------------------------------
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ = msg;
    }

    // --------------------------------------------------
    std::vector<std::pair<double, double>> predict_trajectory(double v, double w)
    {
        std::vector<std::pair<double, double>> traj;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;

        double t = 0.0;
        while (t <= predict_time_)
        {
            x += v * std::cos(yaw) * dt_sim_;
            y += v * std::sin(yaw) * dt_sim_;
            yaw += w * dt_sim_;
            traj.emplace_back(x, y);
            t += dt_sim_;
        }

        return traj;
    }

    // --------------------------------------------------
    double calc_obstacle_cost(const std::vector<std::pair<double, double>> & traj)
    {
        if (!scan_)
        {
            return 0.0;
        }

        double min_dist = std::numeric_limits<double>::infinity();

        for (const auto & point : traj)
        {
            double x = point.first;
            double y = point.second;

            for (size_t i = 0; i < scan_->ranges.size(); ++i)
            {
                double r = scan_->ranges[i];

                if (std::isinf(r) || std::isnan(r))
                {
                    continue;
                }

                double angle = scan_->angle_min + static_cast<double>(i) * scan_->angle_increment;

                double obs_x = r * std::cos(angle);
                double obs_y = r * std::sin(angle);

                double dist = std::hypot(x - obs_x, y - obs_y);

                if (dist < min_dist)
                {
                    min_dist = dist;
                }
            }
        }

        if (std::isinf(min_dist))
        {
            return 0.0;
        }

        // Hard collision only if extremely close
        if (min_dist < robot_radius_)
        {
            return std::numeric_limits<double>::infinity();
        }

        // Smooth penalty
        return 1.0 / (min_dist + 0.01);
    }

    // --------------------------------------------------
    std::pair<double, double> dwa_safety_filter(double v_pid, double w_pid)
    {
        if (!scan_)
        {
            return {v_pid, w_pid};
        }

        double min_v = std::max(-v_max_, v_pid - max_accel_ * dt_sim_);
        double max_v = std::min(v_max_,  v_pid + max_accel_ * dt_sim_);

        // widen angular search around forward motion
        double angle_expand = (std::abs(v_pid) > 0.05) ? 0.7 : 0.4;

        double min_w = std::max(-w_max_, w_pid - max_delta_w_ * dt_sim_ - angle_expand);
        double max_w = std::min(w_max_,  w_pid + max_delta_w_ * dt_sim_ + angle_expand);

        double best_v = 0.0;
        double best_w = 0.0;
        double min_cost = std::numeric_limits<double>::infinity();
        bool found = false;

        // Integer-based sampling to mimic Python np.arange much better
        int n_v = static_cast<int>(std::floor((max_v - min_v) / v_reso_));
        int n_w = static_cast<int>(std::floor((max_w - min_w) / w_reso_));

        for (int i = 0; i <= n_v + 1; ++i)
        {
            double v = min_v + static_cast<double>(i) * v_reso_;
            if (v > max_v + v_reso_ * 0.5)
            {
                continue;
            }

            for (int j = 0; j <= n_w + 1; ++j)
            {
                double w = min_w + static_cast<double>(j) * w_reso_;
                if (w > max_w + w_reso_ * 0.5)
                {
                    continue;
                }

                auto traj = predict_trajectory(v, w);
                double obstacle_cost = calc_obstacle_cost(traj);

                if (std::isinf(obstacle_cost))
                {
                    continue;
                }

                double tracking_cost = std::abs(v - v_pid) + std::abs(w - w_pid);
                double final_cost = 3.5 * obstacle_cost + 4.0 * tracking_cost;

                if (final_cost < min_cost)
                {
                    min_cost = final_cost;
                    best_v = v;
                    best_w = w;
                    found = true;
                }
            }
        }

        // If everything unsafe -> rotate to search free space
        if (!found)
        {
            return {0.0, 0.8};
        }

        return {best_v, best_w};
    }

    // --------------------------------------------------
    void control_loop()
    {
        geometry_msgs::msg::TransformStamped trans;

        try
        {
            trans = tf_buffer_.lookupTransform(
                "base_link",
                "leader/base_link",
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &)
        {
            return;
        }

        double dx = trans.transform.translation.x;
        double dy = trans.transform.translation.y;

        double distance = std::sqrt(dx * dx + dy * dy);

        // ---------- Distance PID ----------
        double error = distance - d_ref_;

        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - prev_time_).nanoseconds() / 1e9;
        if (dt == 0.0)
        {
            return;
        }

        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;

        double v = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
        v = std::max(std::min(v, v_max_), -v_max_);

        // ---------- Heading ----------
        double angle_error = std::atan2(dy, dx);
        double w = Kp_angle_ * angle_error;
        w = std::max(std::min(w, w_max_), -w_max_);

        // ---------- DWA Obstacle Layer ----------
        auto filtered = dwa_safety_filter(v, w);
        v = filtered.first;
        w = filtered.second;

        // ---------- Publish ----------
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = w;
        cmd_pub_->publish(cmd);

        prev_error_ = error;
        prev_time_ = now;

        RCLCPP_INFO(this->get_logger(), "dist=%.2f  v=%.2f  w=%.2f", distance, v, w);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDFollow>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}