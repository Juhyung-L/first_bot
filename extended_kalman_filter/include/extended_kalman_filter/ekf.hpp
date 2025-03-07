#ifndef EKF_HPP_
#define EKF_HPP_

#include <queue>
#include <cmath>
#include <mutex>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"

// Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
// std::string sep{"/-----------------------------------------/"};

const int STATE_SIZE = 8; // x y z vx vy vyaw ax ay
const int ODOM_MEASUREMENT_SIZE = 6; // x y yaw vx vy vyaw
const int IMU_MEASUREMENT_SIZE = 3; // yaw ax ay
const int ODOM_POSE_SIZE = 3; // x y yaw

enum StateMember
{
    StateX = 0,
    StateY,
    StateYaw,
    StateVx,
    StateVy,
    StateVyaw,
    StateAx,
    StateAy
};

enum OdomMember
{
    OdomX = 0,
    OdomY,
    OdomYaw,
    OdomVx,
    OdomVy,
    OdomVyaw
};

enum ImuMember
{
    ImuYaw = 0,
    ImuAx,
    ImuAy
};

enum DataType
{
    Odom = 0,
    Imu
};

struct Measurement
{
    using SharedPtr = std::shared_ptr<Measurement>;
    DataType data_type_;
    rclcpp::Time time_;
    Eigen::VectorXd measurement_;
    Eigen::MatrixXd covariance_;
    Eigen::MatrixXd observation_matrix_;

    // oldest measurement is on top of priority queue
    bool operator()(const Measurement::SharedPtr& a, const Measurement::SharedPtr& b)
    {
        return a->time_ > b->time_;
    }
};

class EKF
{
public:
    EKF()
    {
        state_ = Eigen::Vector<double, STATE_SIZE>::Zero();
        covariance_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
        covariance_ *= 1e-2; // initialize covariance to 0.01

        transfer_function_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
        transfer_function_jacobian_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
    
        identity_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
    }

    void setProcessNosie(const Eigen::MatrixXd& process_noise)
    {
        process_noise_ = process_noise;
    }

    void predictUpdateCycle()
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        while (!measurement_queue_.empty())
        {
            Measurement::SharedPtr msg = measurement_queue_.top();
            measurement_queue_.pop();

            if (cur_time_.nanoseconds() == 0)
            {
                cur_time_ = msg->time_;
                update(msg);
                continue;
            }

            rclcpp::Duration duration = msg->time_ - cur_time_;
            predict(duration);
            update(msg);

            cur_time_ = msg->time_;
        }
    }

    std::priority_queue<Measurement::SharedPtr, std::vector<Measurement::SharedPtr>, Measurement> measurement_queue_;
    Eigen::Vector<double, STATE_SIZE> state_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> covariance_;
    std::mutex state_mtx_;

private:
    rclcpp::Time cur_time_{0};

    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> transfer_function_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> transfer_function_jacobian_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> process_noise_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> identity_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> I_KH_;

    void predict(const rclcpp::Duration& duration)
    {
        // duration = duration since last call to this function
        // this function combines the change in robot's pose since last time stamp given by
        // odom and the kinematic model using the Maximum Likelihood Estimation (MLE)

        // set up transfer function
        double yaw = state_(StateYaw);
        double sin_yaw = std::sin(yaw);
        double cos_yaw = std::cos(yaw);
        double delta_T = duration.seconds();

        transfer_function_(StateX, StateVx) = cos_yaw * delta_T;
        transfer_function_(StateX, StateVy) = -sin_yaw * delta_T;
        transfer_function_(StateX, StateAx) = 0.5 * transfer_function_(StateX, StateVx) * delta_T;
        transfer_function_(StateX, StateAy) = 0.5 * transfer_function_(StateX, StateVy) * delta_T;

        transfer_function_(StateY, StateVx) = sin_yaw * delta_T;
        transfer_function_(StateY, StateVy) = cos_yaw * delta_T;
        transfer_function_(StateY, StateAx) = 0.5 * transfer_function_(StateX, StateVx) * delta_T;
        transfer_function_(StateY, StateAy) = 0.5 * transfer_function_(StateX, StateVy) * delta_T;
    
        transfer_function_(StateYaw, StateVyaw) = delta_T;
        transfer_function_(StateVx, StateAx) = delta_T;
        transfer_function_(StateVy, StateAy) = delta_T;

        // set up transfer function jacobian
        double vx = state_(StateVx);
        double vy = state_(StateVy);
        double ax = state_(StateAx);
        double ay = state_(StateAy);
        double delta_T_sq = delta_T * delta_T;

        transfer_function_jacobian_ = transfer_function_;
        transfer_function_jacobian_(StateX, StateYaw) = 
            -sin_yaw * vx * delta_T - cos_yaw * vy *delta_T - 0.5 * sin_yaw * ax * delta_T_sq - 0.5 * cos_yaw * ay * delta_T_sq;
        transfer_function_jacobian_(StateY, StateYaw) = 
            cos_yaw * vx * delta_T - sin_yaw * vy * delta_T + 0.5 * cos_yaw * ax * delta_T_sq - 0.5 * sin_yaw * ay * delta_T_sq;
        
        // apply transfer function
        state_ = transfer_function_ * state_;

        // update covariance
        covariance_ = 
            transfer_function_jacobian_ * covariance_ * transfer_function_jacobian_.transpose() + process_noise_;
        
        // wrap state angles
        angles::normalize_angle(state_(StateYaw));
        angles::normalize_angle(state_(StateVyaw));
    }

    void update(const Measurement::SharedPtr msg)
    {
        int measurement_size;
        if (msg->data_type_ == Odom) {measurement_size = ODOM_MEASUREMENT_SIZE;}
        else if (msg->data_type_ == Imu) {measurement_size = IMU_MEASUREMENT_SIZE;}
        
        // calculate kalman gain K = (PH') / (HPH' + R)
        Eigen::MatrixXd kalman_gain(STATE_SIZE, measurement_size);
        Eigen::MatrixXd pht = covariance_ * msg->observation_matrix_.transpose();
        Eigen::MatrixXd hphr_inverse = (msg->observation_matrix_ * pht + msg->covariance_).inverse();
        kalman_gain.noalias() = pht * hphr_inverse;

        // update state x = x _ K(z - Hx)
        Eigen::MatrixXd innovation = (msg->measurement_ - msg->observation_matrix_ * state_);
        if (msg->data_type_ == Odom)
        {
            innovation(OdomYaw) = angles::normalize_angle(innovation(OdomYaw));
        }
        else if (msg->data_type_ == Imu)
        {
            innovation(ImuYaw) = angles::normalize_angle(innovation(ImuYaw));
        }
        state_.noalias() += kalman_gain * innovation;

        // update covariance
        I_KH_ = identity_ - kalman_gain * msg->observation_matrix_;
        covariance_ = I_KH_ * covariance_ * I_KH_.transpose();
        covariance_.noalias() += kalman_gain * msg->covariance_ * kalman_gain.transpose();
    }
};

#endif