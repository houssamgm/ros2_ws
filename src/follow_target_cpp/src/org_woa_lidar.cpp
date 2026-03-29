#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <numeric>

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
        // ---------------- Parameters (Identical to Python) ----------------
        d_ref_ = 0.7;
        stop_threshold_ = 0.05;
        v_max_ = 0.6;
        w_max_ = 3.0;
        alpha_ = 2.0;
        beta_ = 1.5;
        n_whales_ = 25;
        max_iter_ = 10;
        dt_ = 0.4;
        b_ = 1.0;

        robot_radius_ = 0.22;
        safety_margin_ = 0.05;

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
        RCLCPP_INFO(this->get_logger(), "WOA follower with HARD safety + LiDAR tracking (Vectorized)");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = msg; }

    // LIDAR PROCESSING
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
            double sum_x = 0, sum_y = 0, min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
            for (const auto& p : cluster) {
                sum_x += p.x; sum_y += p.y;
                min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
            }
            double w = max_y - min_y, l = max_x - min_x;
            double size = std::hypot(w, l);
            if (size >= 0.08 && size <= 0.6 && ((w > 0.08 && w < 0.35) || (l > 0.08 && l < 0.35))) {
                candidates.push_back({sum_x / cluster.size(), sum_y / cluster.size()});
            }
        }

        if (candidates.empty()) {
            if (prev_target_set_ && lost_cycles_ < max_lost_cycles_) {
                lost_cycles_++; return prev_target_;
            }
            return {0.0, 0.0};
        }

        Point best;
        if (!prev_target_set_) {
            lost_cycles_ = 0;
            best = candidates[0];
            double min_d = std::hypot(best.x, best.y);
            for(auto& c : candidates) {
                double d = std::hypot(c.x, c.y);
                if(d < min_d){ min_d = d; best = c; }
            }
        } else {
            std::vector<Point> gated;
            for(auto& c : candidates) {
                if(std::hypot(c.x - prev_target_.x, c.y - prev_target_.y) < track_gate_) gated.push_back(c);
            }
            if (!gated.empty()) {
                lost_cycles_ = 0;
                best = gated[0];
                double min_d = std::hypot(best.x - prev_target_.x, best.y - prev_target_.y);
                for(auto& g : gated) {
                    double d = std::hypot(g.x - prev_target_.x, g.y - prev_target_.y);
                    if(d < min_d){ min_d = d; best = g; }
                }
            } else if (lost_cycles_ < max_lost_cycles_) {
                lost_cycles_++; return prev_target_;
            } else {
                lost_cycles_ = 0;
                best = candidates[0];
                double min_d = std::hypot(best.x, best.y);
                for(auto& c : candidates) {
                    double d = std::hypot(c.x, c.y);
                    if(d < min_d){ min_d = d; best = c; }
                }
            }
        }
        return best;
    }

    // HARD COLLISION CHECK (Identical to Python is_safe)
    bool is_safe(double v, double w) {
        if (!scan_) return true;
        double theta = w * dt_;
        double new_x = v * std::cos(theta);
        double new_y = v * std::sin(theta);
        for (size_t i = 0; i < scan_->ranges.size(); ++i) {
            double r = scan_->ranges[i];
            if (std::isinf(r) || std::isnan(r)) continue;
            double angle = scan_->angle_min + i * scan_->angle_increment;
            if (std::abs(angle) > M_PI / 3.0) continue;
            double obs_x = r * std::cos(angle);
            double obs_y = r * std::sin(angle);
            if (std::hypot(new_x - obs_x, new_y - obs_y) < (robot_radius_ + safety_margin_)) return false;
        }
        return true;
    }

    double fitness(double v, double w, double dx, double dy) {
        if (!is_safe(v, w)) return 1e12;
        double x_rel = dx - v * dt_;
        double th = -w * dt_;
        double dx_p = std::cos(th) * x_rel - std::sin(th) * dy;
        double dy_p = std::sin(th) * x_rel + std::cos(th) * dy;
        return alpha_ * std::abs(std::hypot(dx_p, dy_p) - d_ref_) + beta_ * std::abs(std::atan2(dy_p, dx_p)) + (0.05 * std::abs(v) + 0.02 * std::abs(w));
    }

    // WOA OPTIMIZATION (100% Python Logic with Vectorized A/C)
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

        if (best_whale.fit >= 1e12) return {0.0, 0.8};

        for (int t = 0; t < max_iter_; ++t) {
            double a = 2.0 - 2.0 * (static_cast<double>(t) / max_iter_);
            for (int i = 0; i < n_whales_; ++i) {
                double p = dist_01(gen_), l = dist_m11(gen_);
                
                // Vectorized A and C (independent randoms for v and w)
                double Av = 2.0 * a * dist_01(gen_) - a;
                double Aw = 2.0 * a * dist_01(gen_) - a;
                double Cv = 2.0 * dist_01(gen_);
                double Cw = 2.0 * dist_01(gen_);

                if (p < 0.5) {
                    if (std::hypot(Av, Aw) < 1.0) { // np.linalg.norm(A) < 1.0
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
            }
            for(auto& w : whales) if(w.fit < best_whale.fit) best_whale = w;
            if (best_whale.fit >= 1e12) return {0.0, 0.8};
        }
        return {best_whale.v, best_whale.w};
    }

    void control_loop() {
        if (!scan_) return;
        auto points = scan_to_points();
        Point target = select_target(cluster_points(points));
        if (target.x == 0.0 && target.y == 0.0 && !prev_target_set_) {
            RCLCPP_WARN(this->get_logger(), "Leader not detected"); return;
        }
        prev_target_ = target; prev_target_set_ = true;
        double dist = std::hypot(target.x, target.y), head = std::atan2(target.y, target.x);
        double v_cmd = 0.0, w_cmd = 0.0;
        if (!(std::abs(dist - d_ref_) < stop_threshold_ && std::abs(head) < 0.08)) {
            auto start = std::chrono::high_resolution_clock::now();   // <-- ADD THIS
            auto res = woa_optimize(target.x, target.y);
            auto end = std::chrono::high_resolution_clock::now();     // <-- ADD THIS
            double solve_time_ms = std::chrono::duration<double, std::milli>(end - start).count();  // <-- ADD THIS
            RCLCPP_INFO(this->get_logger(), "WOA solve time = %.3f ms", solve_time_ms);  // <-- ADD THIS
            v_cmd = res.first; w_cmd = res.second;
        }
        prev_v_ = (1.0 - smooth_gain_) * v_cmd + smooth_gain_ * prev_v_;
        prev_w_ = (1.0 - smooth_gain_) * w_cmd + smooth_gain_ * prev_w_;
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = std::clamp(prev_v_, -v_max_, v_max_);
        cmd.angular.z = std::clamp(prev_w_, -w_max_, w_max_);
        cmd_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "dist=%.2f v=%.2f w=%.2f", dist, cmd.linear.x, cmd.angular.z);
    }

    double d_ref_, stop_threshold_, v_max_, w_max_, alpha_, beta_, dt_, b_, smooth_gain_, prev_v_, prev_w_, robot_radius_, safety_margin_, track_gate_;
    int n_whales_, max_iter_, lost_cycles_, max_lost_cycles_;
    bool prev_target_set_; Point prev_target_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::random_device rd_; std::mt19937 gen_;
    // WOA computation time:
    // - Min: 5.14 ms
    // - Max: 10.89 ms
    // - Avg: 7.35 ms
    //
    // → Real-time safe for 10 Hz control loop (100 ms)
    // → CPU usage ≈ 7–11%
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WOAFollow>());
    rclcpp::shutdown();
    return 0;
}