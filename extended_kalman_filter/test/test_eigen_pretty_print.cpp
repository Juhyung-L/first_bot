#include <Eigen/Dense>

int main(void)
{
    Eigen::MatrixXd m(5, 5);
    
    m(1,1) = 2.304;
    m(2,1) = 1999999.99;
    m(4,4) = 1283.0;

    Eigen::VectorXd v(5);
    v(2) = 1283.0;

    Eigen::Matrix<double, 1, 5> v_1;
    v_1(2) = 1273.2;
    return 0;
}