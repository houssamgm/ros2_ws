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
#include <random>

using std::placeholders::_1;

class WOA_Follow : public rclcpp::Node
{
public:
    WOA_Follow()
    : Node("woa_follow"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
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

        prev_v_ = 0.0;
        prev_w_ = 0.0;
        smooth_gain_ = 0.2;

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WOA_Follow::scan_callback, this, _1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WOA_Follow::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "WOA follower with Vectorized A/C coefficients started");
    }

private:
    double d_ref_, stop_threshold_;
    double v_max_, w_max_;
    double alpha_, beta_;
    int n_whales_, max_iter_;
    double dt_, b_;

    double max_accel_, max_delta_w_;
    double predict_time_, dt_sim_;
    double v_reso_, w_reso_;
    double robot_radius_;

    double prev_v_, prev_w_, smooth_gain_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::mt19937 gen_{std::random_device{}()};

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ = msg;
    }

    std::pair<double,double> predict_relative_state(double dx, double dy, double v, double w)
    {
        double x_rel = dx - v * dt_;
        double y_rel = dy;

        double th = -w * dt_;
        double c = std::cos(th);
        double s = std::sin(th);

        double dx_next = c * x_rel - s * y_rel;
        double dy_next = s * x_rel + c * y_rel;

        return {dx_next, dy_next};
    }

    double fitness(const std::vector<double>& X, double dx, double dy)
    {
        double v = X[0];
        double w = X[1];

        auto [dx_p, dy_p] = predict_relative_state(dx, dy, v, w);

        double d = std::hypot(dx_p, dy_p);
        double theta = std::atan2(dy_p, dx_p);

        double e_d = std::abs(d - d_ref_);
        double e_theta = std::abs(theta);

        double effort = 0.05 * std::abs(v) + 0.02 * std::abs(w);

        return alpha_ * e_d + beta_ * e_theta + effort;
    }

    std::pair<double,double> woa_optimize(double dx, double dy)
    {
        std::uniform_real_distribution<double> rand01(0.0, 1.0);
        std::uniform_real_distribution<double> randL(-1.0, 1.0);

        std::vector<std::vector<double>> whales(n_whales_, std::vector<double>(2));

        for (auto &w : whales)
        {
            w[0] = std::uniform_real_distribution<double>(-v_max_, v_max_)(gen_);
            w[1] = std::uniform_real_distribution<double>(-w_max_, w_max_)(gen_);
        }

        std::vector<double> fitness_vals(n_whales_);
        for (int i = 0; i < n_whales_; ++i)
            fitness_vals[i] = fitness(whales[i], dx, dy);

        int best_idx = std::min_element(fitness_vals.begin(), fitness_vals.end()) - fitness_vals.begin();
        std::vector<double> X_best = whales[best_idx];
        double F_best = fitness_vals[best_idx];

        for (int t = 0; t < max_iter_; ++t)
        {
            double a = 2.0 - 2.0 * (double(t) / max_iter_);

            for (int i = 0; i < n_whales_; ++i)
            {
                std::vector<double> Xi = whales[i];
                std::vector<double> X_new(2);

                // --- CHANGE APPLIED HERE: Vectorized A and C ---
                std::vector<double> A(2), C(2);
                for (int d = 0; d < 2; ++d)
                {
                    double r1 = rand01(gen_);
                    double r2 = rand01(gen_);
                    A[d] = 2.0 * a * r1 - a;
                    C[d] = 2.0 * r2;
                }

                double p = rand01(gen_);
                double l = randL(gen_);

                if (p < 0.5)
                {
                    double normA = std::sqrt(A[0]*A[0] + A[1]*A[1]);
                    if (normA < 1.0)
                    {
                        for (int d = 0; d < 2; ++d)
                        {
                            double D = std::abs(C[d] * X_best[d] - Xi[d]);
                            X_new[d] = X_best[d] - A[d] * D;
                        }
                    }
                    else
                    {
                        int rand_idx = std::uniform_int_distribution<int>(0, n_whales_ - 1)(gen_);
                        auto X_rand = whales[rand_idx];

                        for (int d = 0; d < 2; ++d)
                        {
                            double D = std::abs(C[d] * X_rand[d] - Xi[d]);
                            X_new[d] = X_rand[d] - A[d] * D;
                        }
                    }
                }
                else
                {
                    for (int d = 0; d < 2; ++d)
                    {
                        double D = std::abs(X_best[d] - Xi[d]);
                        X_new[d] = D * std::exp(b_ * l) * std::cos(2 * M_PI * l) + X_best[d];
                    }
                }

                X_new[0] = std::clamp(X_new[0], -v_max_, v_max_);
                X_new[1] = std::clamp(X_new[1], -w_max_, w_max_);

                whales[i] = X_new;
            }

            for (int i = 0; i < n_whales_; ++i)
                fitness_vals[i] = fitness(whales[i], dx, dy);

            int idx = std::min_element(fitness_vals.begin(), fitness_vals.end()) - fitness_vals.begin();

            if (fitness_vals[idx] < F_best)
            {
                F_best = fitness_vals[idx];
                X_best = whales[idx];
            }
        }

        return {X_best[0], X_best[1]};
    }

    std::pair<double,double> dwa_safety_filter(double v_cmd, double w_cmd)
    {
        if (!scan_)
            return {v_cmd, w_cmd};

        double min_v = std::max(-v_max_, v_cmd - max_accel_ * dt_sim_);
        double max_v = std::min(v_max_,  v_cmd + max_accel_ * dt_sim_);

        double angle_expand = (std::abs(v_cmd) > 0.05) ? 0.7 : 0.4;

        double min_w = std::max(-w_max_, w_cmd - max_delta_w_ * dt_sim_ - angle_expand);
        double max_w = std::min(w_max_,  w_cmd + max_delta_w_ * dt_sim_ + angle_expand);

        double best_v = 0.0, best_w = 0.0;
        double min_cost = std::numeric_limits<double>::infinity();
        bool found = false;

        int n_v = static_cast<int>(std::floor((max_v - min_v) / v_reso_));
        int n_w = static_cast<int>(std::floor((max_w - min_w) / w_reso_));

        for (int i = 0; i <= n_v + 1; ++i)
        {
            double v = min_v + i * v_reso_;
            if (v > max_v + v_reso_ * 0.5) continue;

            for (int j = 0; j <= n_w + 1; ++j)
            {
                double w = min_w + j * w_reso_;
                if (w > max_w + w_reso_ * 0.5) continue;

                auto traj = predict_trajectory(v, w);
                double obstacle_cost = calc_obstacle_cost(traj);

                if (std::isinf(obstacle_cost))
                    continue;

                double tracking_cost = std::abs(v - v_cmd) + std::abs(w - w_cmd);
                double final_cost = 3.5 * obstacle_cost + 4 * tracking_cost;

                if (final_cost < min_cost)
                {
                    min_cost = final_cost;
                    best_v = v;
                    best_w = w;
                    found = true;
                }
            }
        }

        if (!found)
            return {0.0, 0.8};

        return {best_v, best_w};
    }

    std::vector<std::pair<double,double>> predict_trajectory(double v, double w)
    {
        std::vector<std::pair<double,double>> traj;
        double x=0,y=0,yaw=0;

        for(double t=0;t<=predict_time_;t+=dt_sim_)
        {
            x+=v*std::cos(yaw)*dt_sim_;
            y+=v*std::sin(yaw)*dt_sim_;
            yaw+=w*dt_sim_;
            traj.emplace_back(x,y);
        }
        return traj;
    }

    double calc_obstacle_cost(const std::vector<std::pair<double,double>>& traj)
    {
        if (!scan_) return 0.0;

        double min_dist = std::numeric_limits<double>::infinity();

        for(auto&p:traj)
        {
            for(size_t i=0;i<scan_->ranges.size();++i)
            {
                double r=scan_->ranges[i];
                if(std::isinf(r)||std::isnan(r)) continue;

                double angle=scan_->angle_min+i*scan_->angle_increment;

                double obs_x=r*std::cos(angle);
                double obs_y=r*std::sin(angle);

                double dist=std::hypot(p.first-obs_x,p.second-obs_y);

                if(dist<min_dist) min_dist=dist;
            }
        }

        if(std::isinf(min_dist)) return 0.0;
        if(min_dist<robot_radius_) return std::numeric_limits<double>::infinity();

        return 1.0/(min_dist+0.01);
    }

    void control_loop()
    {
        geometry_msgs::msg::TransformStamped trans;

        try
        {
            trans = tf_buffer_.lookupTransform("base_link","leader/base_link",tf2::TimePointZero);
        }
        catch(...)
        {
            return;
        }

        double dx = trans.transform.translation.x;
        double dy = trans.transform.translation.y;

        double distance = std::hypot(dx, dy);
        double heading = std::atan2(dy, dx);

        double v_cmd, w_cmd;
        auto start = std::chrono::high_resolution_clock::now();   // <-- ADD
        if (std::abs(distance - d_ref_) < stop_threshold_ && std::abs(heading) < 0.08)
        {
            v_cmd = 0.0;
            w_cmd = 0.0;
        }
        else
        {
            auto res = woa_optimize(dx, dy);
            v_cmd = res.first;
            w_cmd = res.second;
        }

        auto filtered = dwa_safety_filter(v_cmd, w_cmd);
        auto end = std::chrono::high_resolution_clock::now();     // <-- ADD
        double solve_time_ms = std::chrono::duration<double, std::milli>(end - start).count();  // <-- ADD
        RCLCPP_INFO(this->get_logger(), "WOA+DWA time = %.3f ms", solve_time_ms);  // <-- ADD
        v_cmd = filtered.first;
        w_cmd = filtered.second;

        double v = (1.0 - smooth_gain_) * v_cmd + smooth_gain_ * prev_v_;
        double w = (1.0 - smooth_gain_) * w_cmd + smooth_gain_ * prev_w_;

        prev_v_ = v;
        prev_w_ = w;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(v, -v_max_, v_max_);
        cmd.angular.z = std::clamp(w, -w_max_, w_max_);

        cmd_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(),"v=%.2f w=%.2f",cmd.linear.x,cmd.angular.z);
        // WOA + DWA computation time:
        // - Min: 11.45 ms
        // - Max: 52.08 ms
        // - Avg: 19.8 ms
        //
        // → Real-time safe for 10 Hz control loop (100 ms)
        // → CPU usage ≈ 20–52%
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WOA_Follow>());
    rclcpp::shutdown();
    return 0;
}