#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

struct Point {
    double x, y;
};

class PIDFollow : public rclcpp::Node {
public:
    PIDFollow() : Node("pid_follow") {
        // ---------- PID parameters ----------
        d_ref_ = 0.7;
        kp_ = 1.2;
        ki_ = 0.0;
        kd_ = 0.2;
        kp_angle_ = 3.0;
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

        // ---------- Detection / tracking fix ----------
        track_gate_ = 0.50;
        max_lost_cycles_ = 6;
        lost_cycles_ = 0;

        // PID memory
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_time_ = this->now();
        prev_target_set_ = false;

        // ROS
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PIDFollow::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&PIDFollow::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "PID + LiDAR + DWA follower (C++) started");
    }

private:
    // --- Callbacks ---
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ = msg;
    }

    // --- LIDAR Processing ---
    std::vector<Point> scan_to_points() {
        std::vector<Point> points;
        if (!scan_) return points;

        for (size_t i = 0; i < scan_->ranges.size(); ++i) {
            double r = scan_->ranges[i];
            if (std::isinf(r) || std::isnan(r)) continue;

            double angle = scan_->angle_min + i * scan_->angle_increment;
            if (std::abs(angle) > 1.2) continue;

            points.push_back({r * std::cos(angle), r * std::sin(angle)});
        }
        return points;
    }

    std::vector<std::vector<Point>> cluster_points(const std::vector<Point>& points, double threshold = 0.15) {
        std::vector<std::vector<Point>> clusters;
        std::vector<bool> used(points.size(), false);

        for (size_t i = 0; i < points.size(); ++i) {
            if (used[i]) continue;

            std::vector<Point> cluster;
            std::vector<size_t> queue;
            
            used[i] = true;
            cluster.push_back(points[i]);
            queue.push_back(i);

            size_t head = 0;
            while (head < queue.size()) {
                Point p = points[queue[head++]];
                for (size_t j = 0; j < points.size(); ++j) {
                    if (used[j]) continue;
                    if (std::hypot(p.x - points[j].x, p.y - points[j].y) < threshold) {
                        used[j] = true;
                        cluster.push_back(points[j]);
                        queue.push_back(j);
                    }
                }
            }
            clusters.push_back(cluster);
        }
        return clusters;
    }

    Point select_target(const std::vector<std::vector<Point>>& clusters) {
        std::vector<Point> candidates;

        for (const auto& cluster : clusters) {
            if (cluster.size() < 5) continue;

            double min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
            double sum_x = 0, sum_y = 0;

            for (const auto& p : cluster) {
                min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
                sum_x += p.x; sum_y += p.y;
            }

            double cx = sum_x / cluster.size();
            double cy = sum_y / cluster.size();
            double w = max_y - min_y;
            double l = max_x - min_x;
            double size = std::hypot(w, l);

            if (size < 0.08 || size > 0.6) continue;
            if (!((w > 0.08 && w < 0.35) || (l > 0.08 && l < 0.35))) continue;

            candidates.push_back({cx, cy});
        }

        if (candidates.empty()) {
            if (prev_target_set_ && lost_cycles_ < max_lost_cycles_) {
                lost_cycles_++;
                return prev_target_;
            }
            return {0.0, 0.0}; // Indicator for no target
        }

        if (!prev_target_set_) {
            lost_cycles_ = 0;
            auto it = std::min_element(candidates.begin(), candidates.end(), [](Point a, Point b){
                return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);
            });
            return *it;
        }

        std::vector<Point> gated;
        for (const auto& c : candidates) {
            if (std::hypot(c.x - prev_target_.x, c.y - prev_target_.y) < track_gate_) {
                gated.push_back(c);
            }
        }

        if (!gated.empty()) {
            lost_cycles_ = 0;
            auto it = std::min_element(gated.begin(), gated.end(), [this](Point a, Point b){
                return std::hypot(a.x - prev_target_.x, a.y - prev_target_.y) < 
                       std::hypot(b.x - prev_target_.x, b.y - prev_target_.y);
            });
            return *it;
        }

        if (lost_cycles_ < max_lost_cycles_) {
            lost_cycles_++;
            return prev_target_;
        }

        lost_cycles_ = 0;
        auto it = std::min_element(candidates.begin(), candidates.end(), [](Point a, Point b){
            return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);
        });
        return *it;
    }

    // --- DWA Functions ---
    double calc_obstacle_cost(const std::vector<Point>& traj) {
        if (!scan_) return 0.0;
        double min_dist = 1e6;

        for (const auto& tp : traj) {
            for (size_t i = 0; i < scan_->ranges.size(); ++i) {
                double r = scan_->ranges[i];
                if (std::isinf(r) || std::isnan(r)) continue;

                double angle = scan_->angle_min + i * scan_->angle_increment;
                double ox = r * std::cos(angle);
                double oy = r * std::sin(angle);

                double d = std::hypot(tp.x - ox, tp.y - oy);
                if (d < min_dist) min_dist = d;
            }
        }

        if (min_dist < robot_radius_) return 1e9; // Infinity equivalent
        return 1.0 / (min_dist + 0.01);
    }

    std::pair<double, double> dwa_safety_filter(double v_pid, double w_pid) {
        if (!scan_) return {v_pid, w_pid};

        double min_v = std::max(-v_max_, v_pid - max_accel_ * dt_sim_);
        double max_v = std::min(v_max_, v_pid + max_accel_ * dt_sim_);
        double angle_expand = (std::abs(v_pid) > 0.05) ? 0.6 : 0.3;
        double min_w = std::max(-w_max_, w_pid - max_delta_w_ * dt_sim_ - angle_expand);
        double max_w = std::min(w_max_, w_pid + max_delta_w_ * dt_sim_ + angle_expand);

        double best_v = 0.0, best_w = 0.8; // Default rotate
        double min_cost = 1e12;
        bool found = false;

        for (double v = min_v; v <= max_v; v += v_reso_) {
            for (double w = min_w; w <= max_w; w += w_reso_) {
                std::vector<Point> traj;
                double tx = 0, ty = 0, tyaw = 0;
                for (double t = 0; t <= predict_time_; t += dt_sim_) {
                    tx += v * std::cos(tyaw) * dt_sim_;
                    ty += v * std::sin(tyaw) * dt_sim_;
                    tyaw += w * dt_sim_;
                    traj.push_back({tx, ty});
                }

                double obs_cost = calc_obstacle_cost(traj);
                if (obs_cost >= 1e9) continue;

                double track_cost = std::abs(v - v_pid) + std::abs(w - w_pid);
                double final_cost = 3.5 * obs_cost + 4.0 * track_cost;

                if (final_cost < min_cost) {
                    min_cost = final_cost;
                    best_v = v; best_w = w;
                    found = true;
                }
            }
        
        }
        if (!found) {
            return {0.0, 0.8};
        }
        return {best_v, best_w};
    }

    void control_loop() {
        if (!scan_) return;

        auto points = scan_to_points();
        auto clusters = cluster_points(points);
        Point target = select_target(clusters);

        if (target.x == 0.0 && target.y == 0.0 && !prev_target_set_) {
            RCLCPP_WARN(this->get_logger(), "Leader not detected");
            return;
        }

        prev_target_ = target;
        prev_target_set_ = true;

        double distance = std::hypot(target.x, target.y);
        double error = distance - d_ref_;

        auto now = this->now();
        double dt = (now - prev_time_).seconds();
        if (dt <= 0) return;

        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;

        double v = kp_ * error + ki_ * integral_ + kd_ * derivative;
        v = std::clamp(v, -v_max_, v_max_);

        double angle_error = std::atan2(target.y, target.x);
        double w = kp_angle_ * angle_error;
        w = std::clamp(w, -w_max_, w_max_);
        auto start = std::chrono::high_resolution_clock::now();
        auto filtered = dwa_safety_filter(v, w);
        auto end = std::chrono::high_resolution_clock::now();
        double solve_time_ms = std::chrono::duration<double, std::milli>(end - start).count();

        RCLCPP_INFO(this->get_logger(), "PID+DWA solve time = %.3f ms", solve_time_ms);
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = filtered.first;
        cmd.angular.z = filtered.second;
        cmd_pub_->publish(cmd);

        prev_error_ = error;
        prev_time_ = now;

        RCLCPP_INFO(this->get_logger(), "dist=%.2f v=%.2f w=%.2f", distance, filtered.first, filtered.second);
        // PID + DWA computation time:
        // - Min: 5.03 ms
        // - Max: 40.37 ms
        // - Avg: 18.9 ms
        //
        // → Real-time safe for 10 Hz control loop (100 ms)
        // → CPU usage ≈ 19–40%
    }

    // Members
    double d_ref_, kp_, ki_, kd_, kp_angle_, v_max_, w_max_;
    double max_accel_, max_delta_w_, predict_time_, dt_sim_, v_reso_, w_reso_, robot_radius_;
    double track_gate_;
    int max_lost_cycles_, lost_cycles_;
    double integral_, prev_error_;
    rclcpp::Time prev_time_;
    Point prev_target_;
    bool prev_target_set_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDFollow>());
    rclcpp::shutdown();
    return 0;
}