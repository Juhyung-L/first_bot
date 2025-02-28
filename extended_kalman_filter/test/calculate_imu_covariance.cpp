#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "msg_2d/msg/imu_stamped.hpp"

// number of data points collected
int n = 0;

// mean
double mean_yaw = 0.0;
double mean_ax = 0.0; 
double mean_ay = 0.0;

// variance
double var_yaw = 0.0;
double var_ax = 0.0;
double var_ay = 0.0;

// covariance
double cov_yaw_ax = 0.0;
double cov_yaw_ay = 0.0;
double cov_ax_ay = 0.0;

void imuCB(const msg_2d::msg::ImuStamped& imu_msg)
{
    ++n;

    double yaw = imu_msg.yaw;
    double ax = imu_msg.acc_x;
    double ay = imu_msg.acc_y;

    // update means
    double dev_from_old_mean_yaw = yaw - mean_yaw;
    double dev_from_old_mean_ax = ax - mean_ax;
    double dev_from_old_mean_ay = ay - mean_ay;

    mean_yaw += dev_from_old_mean_yaw / n;
    mean_ax += dev_from_old_mean_ax / n;
    mean_ay += dev_from_old_mean_ay / n;

    double dev_from_new_mean_yaw = yaw - mean_yaw;
    double dev_from_new_mean_ax = ax - mean_ax;
    double dev_from_new_mean_ay = ay - mean_ay;

    // update variance
    var_yaw += dev_from_old_mean_yaw * dev_from_new_mean_yaw;
    var_ax += dev_from_old_mean_ax * dev_from_new_mean_ax;
    var_ay += dev_from_old_mean_ay * dev_from_new_mean_ay;

    // update covariance
    cov_yaw_ax += dev_from_new_mean_yaw * dev_from_new_mean_ax;
    cov_yaw_ay += dev_from_new_mean_yaw * dev_from_new_mean_ay;
    cov_ax_ay += dev_from_new_mean_ax * dev_from_new_mean_ay;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dummy_node");
    auto imu_sub = node->create_subscription<msg_2d::msg::ImuStamped>(
        "imu", rclcpp::SystemDefaultsQoS(), std::bind(&imuCB, std::placeholders::_1));
    rclcpp::spin(node);

    // after user presses Ctrl^C, need to divide variances and covariances by n-1
    var_yaw /= (n-1);
    var_ax /= (n-1);
    var_ay /= (n-1);
    cov_yaw_ax /= (n-1);
    cov_yaw_ay /= (n-1);
    cov_ax_ay /= (n-1);

    // save covariance matrix to file
    std::string file_name = "/home/dev_ws/src/real_launch/config/imu_covariance.txt";
    std::ofstream out_file(file_name, std::ios::app);
    if (out_file.is_open())
    {
        out_file << '[' 
        << var_yaw    << ", " << cov_yaw_ax << ", " << cov_yaw_ay << ", "
        << cov_yaw_ax << ", " << var_ax     << ", " << cov_ax_ay << ", "
        << cov_yaw_ay << ", " << cov_ax_ay  << ", " << var_ay
        << "]\n";
        out_file.close();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Could not open file %s", file_name.c_str());
    }

    rclcpp::shutdown();
    return 0;
}