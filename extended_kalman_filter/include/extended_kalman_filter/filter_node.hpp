#ifndef FILTER_NODE_HPP_
#define FILTER_NODE_HPP_

#include <chrono>
#include <memory>
#include <queue>
#include <mutex>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "msg_2d/msg/imu_stamped.hpp"
#include "msg_2d/msg/odometry_stamped.hpp"
#include "msg_2d/msg/pose.hpp"
#include "extended_kalman_filter/ekf.hpp"

Eigen::Matrix<double, ODOM_MEASUREMENT_SIZE, ODOM_MEASUREMENT_SIZE> ODOM_COVARIANCE;
Eigen::Matrix<double, IMU_MEASUREMENT_SIZE, IMU_MEASUREMENT_SIZE> IMU_COVARIANCE;
Eigen::Matrix<double, ODOM_MEASUREMENT_SIZE, STATE_SIZE> ODOM_OBSERVATION_MATRIX;
Eigen::Matrix<double, IMU_MEASUREMENT_SIZE, STATE_SIZE> IMU_OBSERVATION_MATRIX;

class FilterNode : public rclcpp::Node
{
public:
    // this filter will take in:
    // - delta x, y, yaw, vx, vy, and vyaw from wheel odometry
    // - accel x, accel y, and yaw from IMU
    FilterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("filter_node", options)
    {
        declare_parameter("odom_topic", "odom_delta");
        declare_parameter("imu_topic", "imu");
        declare_parameter("filtered_pose_topic", "filtered_pose");

        declare_parameter("odom_covariance", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
        declare_parameter("imu_covariance", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);

        declare_parameter("filter_period_ms", 10);
        declare_parameter("process_noise", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);

        std::vector<double> odom_covariance_vector = get_parameter("odom_covariance").as_double_array();
        std::vector<double> imu_covariance_vector = get_parameter("imu_covariance").as_double_array();
        std::vector<double> process_noise_vector = get_parameter("process_noise").as_double_array();

        // set covariance matrices
        for (int i=0; i<ODOM_MEASUREMENT_SIZE; ++i)
        {
            for (int j=0; j<ODOM_MEASUREMENT_SIZE; ++j)
            {
                ODOM_COVARIANCE(i, j) = odom_covariance_vector[i*ODOM_MEASUREMENT_SIZE + j];
            }
        }
        for (int i=0; i<IMU_MEASUREMENT_SIZE; ++i)
        {
            for (int j=0; j<IMU_MEASUREMENT_SIZE; ++j)
            {
                IMU_COVARIANCE(i, j) = imu_covariance_vector[i*IMU_MEASUREMENT_SIZE + j];
            }
        }
        
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> process_noise;
        for (int i=0; i<STATE_SIZE; ++i)
        {
            for (int j=0; j<STATE_SIZE; ++j)
            {
                process_noise(i, j) = process_noise_vector[i*STATE_SIZE + j];
            }
        }
        ekf_.setProcessNosie(process_noise);

        // set observation matrices
        ODOM_OBSERVATION_MATRIX = Eigen::Matrix<double, ODOM_MEASUREMENT_SIZE, STATE_SIZE>::Zero();
        ODOM_OBSERVATION_MATRIX(OdomX, StateX) = 1.0;
        ODOM_OBSERVATION_MATRIX(OdomY, StateY) = 1.0;
        ODOM_OBSERVATION_MATRIX(OdomYaw, StateYaw) = 1.0;
        ODOM_OBSERVATION_MATRIX(OdomVx, StateVx) = 1.0;
        ODOM_OBSERVATION_MATRIX(OdomVy, StateVy) = 1.0;
        ODOM_OBSERVATION_MATRIX(OdomVyaw, StateVyaw) = 1.0;
        IMU_OBSERVATION_MATRIX = Eigen::Matrix<double, IMU_MEASUREMENT_SIZE, STATE_SIZE>::Zero();
        IMU_OBSERVATION_MATRIX(ImuYaw, StateYaw) = 1.0;
        IMU_OBSERVATION_MATRIX(ImuAx, StateAx) = 1.0;
        IMU_OBSERVATION_MATRIX(ImuAy, StateAy) = 1.0;

        std::string odom_topic_ = get_parameter("odom_topic").as_string();
        std::string imu_topic_ = get_parameter("imu_topic").as_string();
        std::string filtered_pose_topic_ = get_parameter("filtered_pose_topic").as_string();
        
        odom_sub_ = create_subscription<msg_2d::msg::OdometryStamped>(
            odom_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&FilterNode::odomMeasurementCB, this, std::placeholders::_1));
        imu_sub_ = create_subscription<msg_2d::msg::ImuStamped>(
            imu_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&FilterNode::imuMeasurementCB, this, std::placeholders::_1));
        
        filtered_pose_pub_ = create_publisher<msg_2d::msg::Pose>(
            filtered_pose_topic_, rclcpp::SystemDefaultsQoS());

        int filter_period_ms = get_parameter("filter_period_ms").as_int();
        timer_ = create_wall_timer(std::chrono::milliseconds(filter_period_ms), std::bind(&FilterNode::updateAndPublish, this));
    }

private:

    rclcpp::Subscription<msg_2d::msg::OdometryStamped>::SharedPtr odom_sub_;
    rclcpp::Subscription<msg_2d::msg::ImuStamped>::SharedPtr imu_sub_;
    rclcpp::Publisher<msg_2d::msg::Pose>::SharedPtr filtered_pose_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    EKF ekf_;

    void odomMeasurementCB(const msg_2d::msg::OdometryStamped& odom_msg)
    {
        // becuase this odom is the change in position since last measurement (delta)
        // we need to add the filter's current state to the measurement
        // as well as the state's covariance  to the measurement covariance
        std::lock_guard<std::mutex> lock(ekf_.state_mtx_);

        Measurement::SharedPtr msg = std::make_shared<Measurement>();
        msg->measurement_ = Eigen::Vector<double, ODOM_MEASUREMENT_SIZE>();

        // convert delta x and y to world frame
        double sin_yaw = std::sin(ekf_.state_(StateYaw));
        double cos_yaw = std::cos(ekf_.state_(StateYaw));
        double x = odom_msg.pose.x*cos_yaw - odom_msg.pose.y*sin_yaw;
        double y = odom_msg.pose.x*sin_yaw + odom_msg.pose.y*cos_yaw;

        msg->measurement_(OdomX) = x + ekf_.state_(StateX);
        msg->measurement_(OdomY) = y + ekf_.state_(StateY);
        msg->measurement_(OdomYaw) = odom_msg.pose.yaw + ekf_.state_(StateYaw);
        msg->measurement_(OdomVx) = odom_msg.velocity.vx;
        msg->measurement_(OdomVy) = odom_msg.velocity.vy;
        msg->measurement_(OdomVyaw) = odom_msg.velocity.vyaw;

        msg->covariance_ = ODOM_COVARIANCE;
        msg->covariance_.block<ODOM_POSE_SIZE, ODOM_POSE_SIZE>(0, 0) += 
            ekf_.covariance_.block<ODOM_POSE_SIZE, ODOM_POSE_SIZE>(0, 0);
        msg->observation_matrix_ = ODOM_OBSERVATION_MATRIX;
        msg->data_type_ = Odom;
        msg->time_ = odom_msg.header.stamp;

        ekf_.measurement_queue_.push(msg);
    }

    void imuMeasurementCB(const msg_2d::msg::ImuStamped& imu_msg)
    {
        Measurement::SharedPtr msg = std::make_shared<Measurement>();
        msg->measurement_ = Eigen::Vector<double, IMU_MEASUREMENT_SIZE>();
        msg->measurement_(ImuYaw) = imu_msg.yaw;
        msg->measurement_(ImuAx) = imu_msg.acc_x;
        msg->measurement_(ImuAy) = imu_msg.acc_y;

        msg->covariance_ = IMU_COVARIANCE;
        msg->observation_matrix_ = IMU_OBSERVATION_MATRIX;
        msg->data_type_ = Imu;
        msg->time_ = imu_msg.header.stamp;

        ekf_.measurement_queue_.push(msg);
    }

    void updateAndPublish()
    {
        ekf_.predictUpdateCycle();

        msg_2d::msg::Pose cur_pose;
        cur_pose.x = ekf_.state_(StateX);
        cur_pose.y = ekf_.state_(StateY);
        cur_pose.yaw = ekf_.state_(StateYaw);

        filtered_pose_pub_->publish(cur_pose);
    }
};

#endif