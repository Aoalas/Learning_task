//
// Created by eoeles on 25-2-9.
//
#include "buff_controller.h"

namespace buff_controller {
    bool BuffController::init(hardware_interface::EffortJointInterface *hw,
                              ros::NodeHandle &nh_) {

        try {
            // 加载参数
            nh_.param("feedforward/enable", use_feedforward_, false);
            nh_.param("feedforward/k_f", k_f_, 0.5);
            nh_.param("pid/p", Kp_, 1.0);
            nh_.param("pid/i", Ki_, 0.1);
            nh_.param("pid/d", Kd_, 0.0);

            // 初始化订阅和发布
            mode_sub_ = nh_.subscribe("/cmd_mode", 1, &BuffController::modeCallback, this);
            target_pub_ = nh_.advertise<std_msgs::Float64>("/target", 10);
            error_pub_ = nh_.advertise<std_msgs::Float64>("/error", 10);

            // 初始化随机数生成器
            gen_.seed(std::random_device()());
            a_dist_ = std::uniform_real_distribution<>(0.780, 1.045);
            omega_dist_ = std::uniform_real_distribution<>(1.884, 2.000);

            a_ = a_dist_(gen_);
            omega_ = omega_dist_(gen_);
            b_ = 2.090 - a_;

            motor_joint_ = hw->getHandle("wheel_joint");
            pid_controller_.initPid(Kp_, Ki_, Kd_, 100.0, -100.0);

            if (use_feedforward_){
                ROS_INFO("Feedforward ON.");
            } else {
                ROS_INFO("Feedforward OFF.");
            }

            return true;

        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM("Failed initialized : " << e.what());
            return false;
        }

    }

    void BuffController::update(const ros::Time &time, const ros::Duration &period) {
        controlSpeed(time,period);
    }

    void BuffController::modeCallback(const std_msgs::Int32::ConstPtr &mode_msg) {
        mode = mode_msg->data;
        if (mode != last_mode_) {
            record_time_ = ros::Time::now().toSec(); // 记录当前时间
        }
        last_mode_ = mode;
    }

    void BuffController::controlSpeed(const ros::Time &time, const ros::Duration &period) {
        if (mode == 0) {
            target_vel_ = 10.0 * 2 * M_PI / 60.0; // 10RPM转rad/s
        } else {
            double t = time.toSec();
            target_vel_ = a_ * sin(omega_ * (t - record_time_)) + b_;
        }

        // PID控制
        current_vel_ = motor_joint_.getVelocity();
        error = target_vel_ - current_vel_;

        error_msg.data = error;
        target_msg.data = target_vel_;

        // 发布控制量
        target_pub_.publish(target_msg);
        error_pub_.publish(error_msg);

        if (use_feedforward_) {
            uff_error_ = target_vel_ - last_vel_ ;
            double uff = Kp_ * uff_error_;
            last_vel_ = target_vel_;
            double effort = pid_controller_.computeCommand(error, period);
            motor_joint_.setCommand(uff + effort);

        } else {
            // PID
            double effort = pid_controller_.computeCommand(error, period);
            motor_joint_.setCommand(effort);

        }
    }
}
PLUGINLIB_EXPORT_CLASS(buff_controller::BuffController, controller_interface::ControllerBase)
