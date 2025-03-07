#include <iostream>

#include <Eigen/Dense>

Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");

int main(int argc, char** argv)
{
    const int state_size = 8;
    const int measurement_size = 3;

    Eigen::Vector<double, state_size> state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix<double, state_size, state_size> covariance;
    covariance <<
        0.01014,  0.01014,  0.01014,  0.03062,     0.01, 0.008303,  0.01021,     0.01,
        0.01014,  0.01014,  0.01014,     0.01,  0.03062, 0.008303,  0.01021,     0.01,
        0.01014,  0.01014,  0.01014,  -0.6324,  -0.6324,  0.03062, -0.01619,  0.01625,
        0.01014,  0.01014,  0.01014,  -0.6324,  -0.6324, 0.008303,  0.03062,  0.01625,
        0.01014,  0.01014,  0.01014,  -0.6324,  -0.6324, 0.008303, -0.01619,  0.03062,
        0.01014,  0.01014,  0.01014,  -0.6324,  -0.6324, 0.008303, -0.01619,  0.01625,
        0.01014,  0.01014,  0.01014,  -0.6324,  -0.6324, 0.008303, -0.01619,  0.01625,
        0.01014,  0.01014,  0.01014,  -0.6324,  -0.6324, 0.008303, -0.01619,  0.01625;
    
    
    Eigen::Vector<double, measurement_size> measurement{0.0001225, 0.009577, 0.02753};
    Eigen::Matrix<double, measurement_size, measurement_size> measurement_covariance;
    measurement_covariance << 
        0.166229, -0.00015777, 0.000309815, -0.00015777, 3.06851e-05, 7.11287e-08, 0.000309815, 7.11287e-08, 6.31037e-05;

    Eigen::Matrix<double, state_size, measurement_size> kalman_gain;
    Eigen::Matrix<double, measurement_size, state_size> observation_matrix;
    observation_matrix.setZero();
    observation_matrix(0, 2) = 1.0;
    observation_matrix(1, 6) = 1.0;
    observation_matrix(2, 7) = 1.0; 
    Eigen::Matrix<double, state_size, measurement_size> observation_matrix_T = observation_matrix.transpose();

    Eigen::MatrixXd a = covariance * observation_matrix_T;
    Eigen::MatrixXd b = observation_matrix * covariance * observation_matrix_T;
    Eigen::MatrixXd c = b + measurement_covariance;
    Eigen::MatrixXd d = c.inverse();
    kalman_gain = a * d;

    state = state + kalman_gain * (measurement - observation_matrix * state);

    std::cout << "State" << std::endl;
    std::cout << state.format(fmt) << std::endl;

    return 0;
}