#ifndef EKF_HPP_
#define EKF_HPP_

#include <iostream>
#include <queue>
#include <cmath>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
std::string sep{"/-----------------------------------------/"};

const int STATE_SIZE = 8; // x y z vx vy vyaw ax ay
const int ODOM_MEASUREMENT_SIZE = 3; // x y yaw
const int IMU_MEASUREMENT_SIZE = 3; // yaw ax ay

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
        covariance_ = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Ones();
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
        while (!measurement_queue_.empty())
        {
            std::cout << "Iteration: " << count <<std::endl; // TODO delete after debug
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
            ++count; // TODO delete after debug
        }
    }

    Eigen::VectorXd getState()
    {
        return state_;
    }

    std::priority_queue<Measurement::SharedPtr, std::vector<Measurement::SharedPtr>, Measurement> measurement_queue_;
private:
    rclcpp::Time cur_time_{0};
    int count {0}; // TODO delete after debug

    Eigen::Vector<double, STATE_SIZE> state_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> covariance_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> transfer_function_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> transfer_function_jacobian_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> process_noise_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> identity_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> I_KH_;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> I_KH_T_;

    void predict(const rclcpp::Duration& duration)
    {
        // set up transfer function
        double yaw = state_(StateYaw);
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        double delta_T = duration.seconds();

        covariance_(StateX, StateVx) = cos_yaw * delta_T;
        covariance_(StateX, StateVy) = -sin_yaw * delta_T;
        covariance_(StateX, StateAx) = 0.5 * covariance_(StateX, StateVx) * delta_T;
        covariance_(StateX, StateAy) = 0.5 * covariance_(StateX, StateVy) * delta_T;

        covariance_(StateY, StateVx) = sin_yaw * delta_T;
        covariance_(StateY, StateVy) = cos_yaw * delta_T;
        covariance_(StateY, StateAx) = 0.5 * covariance_(StateX, StateVx) * delta_T;
        covariance_(StateY, StateAy) = 0.5 * covariance_(StateX, StateVy) * delta_T;
    
        covariance_(StateYaw, StateVyaw) = delta_T;
        covariance_(StateVx, StateAx) = delta_T;
        covariance_(StateVy, StateAy) = delta_T;

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
        covariance_ = transfer_function_jacobian_ * covariance_ * transfer_function_jacobian_.transpose() + process_noise_;
        
        // wrap angles
        state_(StateYaw) = wrapAngle(state_(StateYaw));
        state_(StateVyaw) = wrapAngle(state_(StateVyaw));

        // TODO delete after debug
        std::cout << "Predict" << sep << std::endl;
        std::cout << "Delta T: " << delta_T << std::endl;
        std::cout << "State: \n";
        std::cout << state_.format(fmt) << std::endl;
        std::cout << "Covariance: \n";
        std::cout << covariance_.format(fmt) << std::endl;
        std::cout << sep << std::endl;
    }

    void update(const Measurement::SharedPtr msg)
    {
        int measurement_size;
        if (msg->data_type_ == Odom) {measurement_size = ODOM_MEASUREMENT_SIZE;}
        else if (msg->data_type_ == Imu) {measurement_size = IMU_MEASUREMENT_SIZE;}
        
        // calculate kalman gain
        Eigen::MatrixXd kalman_gain(STATE_SIZE, measurement_size);
        Eigen::MatrixXd observation_transpose = msg->observation_matrix_.transpose();
        kalman_gain = 
            covariance_ * observation_transpose * (msg->observation_matrix_ * covariance_ * observation_transpose + msg->covariance_).inverse();
    
        // update state
        state_ = state_ + kalman_gain * (msg->measurement_ - msg->observation_matrix_ * state_);

        // update covariance
        I_KH_ = identity_ - kalman_gain * msg->observation_matrix_;
        I_KH_T_ = I_KH_.transpose();
        covariance_ = I_KH_ * covariance_ * I_KH_T_ + kalman_gain * msg->covariance_ * kalman_gain.transpose();
        
        // TODO delete after debug
        std::string data_type = (msg->data_type_==Odom) ? "Odom" : "Imu";
        std::cout << "Update" << sep << std::endl;
        std::cout << "Measurement {" << data_type << "}: \n";
        std::cout << msg->measurement_.format(fmt) << std::endl;
        std::cout << "State: \n";
        std::cout << state_.format(fmt) << std::endl;
        std::cout << "Covariance: \n";
        std::cout << covariance_.format(fmt) << std::endl;
        std::cout << sep << std::endl;
    }

    // wrap anglesto [-pi, pi]
    double wrapAngle(double rad)
    {
        constexpr double _2_PI = 2* M_PI;
        rad = std::fmod(rad + M_PI, _2_PI);
        if (rad < 0.0)
        {
            rad += _2_PI;
        }
        return rad - M_PI;
    }

};

#endif