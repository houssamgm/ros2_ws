#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

struct Point {
    double x, y;
};

class WOAFollow : public rclcpp::Node {
public:
    WOAFollow() : Node("woa_follow"), gen_(rd_()) {
        // ---------------- Parameters (Exact Match) ----------------
        d_ref_ = 0.7;
        stop_threshold_ = 0.05;
        v_max_ = 0.6;
        w_max_ = 3.0;
        alpha_ = 2.0;
        beta_ = 1.5;
        n_whales_ = 25;
        max_iter_ = 10;
        dt_ = 0.6;
        b_ = 1.0;

        max_accel_ = 0.6;
        max_delta_w_ = 1.5;
        predict_time_ = 0.8;
        dt_sim_ = 0.1;
        v_reso_ = 0.03;
        w_reso_ = 0.10;
        robot_radius_ = 0.22;

        track_gate_ = 0.50;
        max_lost_cycles_ = 6;
        lost_cycles_ = 0;
        prev_target_set_ = false;

        prev_v_ = 0.0;
        prev_w_ = 0.0;
        smooth_gain_ = 0.2;

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WOAFollow::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&WOAFollow::control_loop, this));
        RCLCPP_INFO(this->get_logger(), "Vectorized WOA Follower Started");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = msg; }

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

    std::vector<std::vector<Point>> cluster_points(const std::vector<Point>& points) {
        std::vector<std::vector<Point>> clusters;
        std::vector<bool> used(points.size(), false);
        for (size_t i = 0; i < points.size(); ++i) {
            if (used[i]) continue;
            std::vector<Point> cluster;
            std::vector<size_t> queue = {i};
            used[i] = true;
            size_t head = 0;
            while (head < queue.size()) {
                Point p = points[queue[head++]];
                cluster.push_back(p);
                for (size_t j = 0; j < points.size(); ++j) {
                    if (!used[j] && std::hypot(p.x - points[j].x, p.y - points[j].y) < 0.15) {
                        used[j] = true; queue.push_back(j);
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
            double min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6, sum_x = 0, sum_y = 0;
            for (const auto& p : cluster) {
                min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
                sum_x += p.x; sum_y += p.y;
            }
            double w = max_y - min_y, l = max_x - min_x, size = std::hypot(w, l);
            if (size < 0.08 || size > 0.6) continue;
            if (!((w > 0.08 && w < 0.35) || (l > 0.08 && l < 0.35))) continue;
            candidates.push_back({sum_x / cluster.size(), sum_y / cluster.size()});
        }
        if (candidates.empty()) {
            if (prev_target_set_ && lost_cycles_ < max_lost_cycles_) {
                lost_cycles_++; return prev_target_;
            }
            return {0.0, 0.0};
        }
        Point best_cand = candidates[0];
        double min_dist_val = 1e6;
        for (const auto& c : candidates) {
            double d = prev_target_set_ ? std::hypot(c.x - prev_target_.x, c.y - prev_target_.y) : std::hypot(c.x, c.y);
            if (d < min_dist_val) { min_dist_val = d; best_cand = c; }
        }
        if (prev_target_set_ && min_dist_val > track_gate_) {
            if (lost_cycles_ < max_lost_cycles_) { lost_cycles_++; return prev_target_; }
        }
        lost_cycles_ = 0; return best_cand;
    }

    double fitness(double v, double w, double dx, double dy) {
        double x_rel = dx - v * dt_;
        double th = -w * dt_;
        double dx_p = std::cos(th) * x_rel - std::sin(th) * dy;
        double dy_p = std::sin(th) * x_rel + std::cos(th) * dy;
        double d = std::hypot(dx_p, dy_p);
        double theta = std::atan2(dy_p, dx_p);
        return alpha_ * std::abs(d - d_ref_) + beta_ * std::abs(theta) + (0.05 * std::abs(v) + 0.02 * std::abs(w));
    }

    std::pair<double, double> woa_optimize(double dx, double dy) {
        std::uniform_real_distribution<double> dist_v(-v_max_, v_max_);
        std::uniform_real_distribution<double> dist_w(-w_max_, w_max_);
        std::uniform_real_distribution<double> dist_01(0.0, 1.0);
        std::uniform_real_distribution<double> dist_m11(-1.0, 1.0);

        struct Whale { double v, w, fit; };
        std::vector<Whale> whales(n_whales_);
        Whale best_whale = {0, 0, 1e12};

        for (int i = 0; i < n_whales_; ++i) {
            whales[i].v = dist_v(gen_); whales[i].w = dist_w(gen_);
            whales[i].fit = fitness(whales[i].v, whales[i].w, dx, dy);
            if (whales[i].fit < best_whale.fit) best_whale = whales[i];
        }

        for (int t = 0; t < max_iter_; ++t) {
            double a = 2.0 - 2.0 * (static_cast<double>(t) / max_iter_);
            for (int i = 0; i < n_whales_; ++i) {
                double p = dist_01(gen_), l = dist_m11(gen_);
                
                // VECTORIZED COEFFICIENTS (Independent for v and w)
                double r1_v = dist_01(gen_), r1_w = dist_01(gen_);
                double r2_v = dist_01(gen_), r2_w = dist_01(gen_);
                
                double Av = 2.0 * a * r1_v - a;
                double Aw = 2.0 * a * r1_w - a;
                double Cv = 2.0 * r2_v;
                double Cw = 2.0 * r2_w;

                if (p < 0.5) {
                    // Check L2 Norm of the A vector
                    if (std::sqrt(Av*Av + Aw*Aw) < 1.0) {
                        whales[i].v = best_whale.v - Av * std::abs(Cv * best_whale.v - whales[i].v);
                        whales[i].w = best_whale.w - Aw * std::abs(Cw * best_whale.w - whales[i].w);
                    } else {
                        int r_idx = std::uniform_int_distribution<int>(0, n_whales_ - 1)(gen_);
                        whales[i].v = whales[r_idx].v - Av * std::abs(Cv * whales[r_idx].v - whales[i].v);
                        whales[i].w = whales[r_idx].w - Aw * std::abs(Cw * whales[r_idx].w - whales[i].w);
                    }
                } else {
                    double spiral = std::exp(b_ * l) * std::cos(2.0 * M_PI * l);
                    whales[i].v = std::abs(best_whale.v - whales[i].v) * spiral + best_whale.v;
                    whales[i].w = std::abs(best_whale.w - whales[i].w) * spiral + best_whale.w;
                }

                whales[i].v = std::clamp(whales[i].v, -v_max_, v_max_);
                whales[i].w = std::clamp(whales[i].w, -w_max_, w_max_);
                whales[i].fit = fitness(whales[i].v, whales[i].w, dx, dy);
                if (whales[i].fit < best_whale.fit) best_whale = whales[i];
            }
        }
        return {best_whale.v, best_whale.w};
    }

    double calc_obstacle_cost(double v, double w) {
        if (!scan_) return 0.0;
        double min_d = 1e6;
        double tx = 0, ty = 0, tyaw = 0;
        for (double t = 0; t <= predict_time_; t += dt_sim_) {
            tx += v * std::cos(tyaw) * dt_sim_;
            ty += v * std::sin(tyaw) * dt_sim_;
            tyaw += w * dt_sim_;
            for (size_t i = 0; i < scan_->ranges.size(); ++i) {
                double r = scan_->ranges[i];
                if (std::isinf(r) || std::isnan(r)) continue;
                double ang = scan_->angle_min + i * scan_->angle_increment;
                double d = std::hypot(tx - r * std::cos(ang), ty - r * std::sin(ang));
                if (d < min_d) min_d = d;
            }
        }
        if (min_d < robot_radius_) return 1e9;
        return 1.0 / (min_d + 0.01);
    }

    std::pair<double, double> dwa_safety_filter(double v_cmd, double w_cmd) {
        if (!scan_) return {v_cmd, w_cmd};
        double min_v = std::max(-v_max_, v_cmd - max_accel_ * dt_sim_);
        double max_v = std::min(v_max_, v_cmd + max_accel_ * dt_sim_);
        double min_w = std::max(-w_max_, w_cmd - max_delta_w_ * dt_sim_ - 0.4);
        double max_w = std::min(w_max_, w_cmd + max_delta_w_ * dt_sim_ + 0.4);

        double best_v = 0.0, best_w = 0.8, min_cost = 1e12;
        bool found = false;
        for (double v = min_v; v <= max_v + 0.001; v += v_reso_) {
            for (double w = min_w; w <= max_w + 0.001; w += w_reso_) {
                double obs_cost = calc_obstacle_cost(v, w);
                if (obs_cost >= 1e9) continue;
                double cost = 3.5 * obs_cost + 4.0 * (std::abs(v - v_cmd) + std::abs(w - w_cmd));
                if (cost < min_cost) { min_cost = cost; best_v = v; best_w = w; found = true; }
            }
        }
        return found ? std::make_pair(best_v, best_w) : std::make_pair(0.0, 0.8);
    }

    void control_loop() {
        if (!scan_) return;
        auto points = scan_to_points();
        auto clusters = cluster_points(points);
        Point target = select_target(clusters);

        if (target.x == 0.0 && target.y == 0.0 && !prev_target_set_) {
            RCLCPP_WARN(this->get_logger(), "Leader LOST"); return;
        }

        prev_target_ = target; prev_target_set_ = true;
        double dist = std::hypot(target.x, target.y), head = std::atan2(target.y, target.x);
        double v_cmd = 0.0, w_cmd = 0.0;

        if (!(std::abs(dist - d_ref_) < stop_threshold_ && std::abs(head) < 0.08)) {
            auto res = woa_optimize(target.x, target.y);
            v_cmd = res.first; w_cmd = res.second;
        }

        auto filtered = dwa_safety_filter(v_cmd, w_cmd);
        double v = (1.0 - smooth_gain_) * filtered.first + smooth_gain_ * prev_v_;
        double w = (1.0 - smooth_gain_) * filtered.second + smooth_gain_ * prev_w_;
        prev_v_ = v; prev_w_ = w;

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = std::clamp(v, -v_max_, v_max_);
        cmd.angular.z = std::clamp(w, -w_max_, w_max_);

        // DISPLAY OUTPUT
        RCLCPP_INFO(this->get_logger(), "D: %.2f | V: %.2f | W: %.2f", dist, cmd.linear.x, cmd.angular.z);

        cmd_pub_->publish(cmd);
    }

    double d_ref_, stop_threshold_, v_max_, w_max_, alpha_, beta_, dt_, b_, smooth_gain_, prev_v_, prev_w_;
    int n_whales_, max_iter_, lost_cycles_, max_lost_cycles_;
    double track_gate_, max_accel_, max_delta_w_, predict_time_, dt_sim_, v_reso_, w_reso_, robot_radius_;
    bool prev_target_set_; Point prev_target_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::random_device rd_; std::mt19937 gen_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WOAFollow>());
    rclcpp::shutdown();
    return 0;
}