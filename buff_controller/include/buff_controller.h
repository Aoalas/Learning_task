//
// Created by eoeles on 25-2-16.
//

#ifndef BUFF_CONTROLLER_BUFF_CONTROLLER_H
#define BUFF_CONTROLLER_BUFF_CONTROLLER_H

#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <cstdlib>
#include <random>
#include <control_toolbox/pid.h>
#include <pluginlib/class_list_macros.hpp>
#include <memory>
#include <controller_interface/multi_interface_controller.h>
#include <rm_common/hardware_interface/robot_state_interface.h>

namespace buff_controller {
    class BuffController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

    public:
        BuffController() = default;
        ~BuffController() override = default;

        bool init(hardware_interface::EffortJointInterface *hw,
                  ros::NodeHandle &nh_) override;

        void modeCallback(const std_msgs::Int32::ConstPtr &msg);

        void controlSpeed(const ros::Time &time, const ros::Duration &period);

        void update(const ros::Time &time, const ros::Duration &period) override;

    private:
        ros::Subscriber mode_sub_;
        ros::Publisher effort_pub_, target_pub_, current_pub_,error_pub_;

        int mode = 0;
        bool use_feedforward_;
        double k_f_, Kp_, Ki_, Kd_;
        double a_, omega_, b_;
        double record_time_ = 0;
        double last_vel_ = 0;
        double uff_error_ = 0;
        int last_mode_ = 0;
        std_msgs::Float64 msg;
        std_msgs::Float64 error_msg;
        std_msgs::Float64 target_msg;
        double error;
        double target_vel_,current_vel_;

        hardware_interface::JointHandle motor_joint_;
        control_toolbox::Pid pid_controller_;

        std::default_random_engine gen_;
        std::uniform_real_distribution<double> a_dist_, omega_dist_;
    };
}
#endif //BUFF_CONTROLLER_BUFF_CONTROLLER_H
