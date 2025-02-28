add_executable(calculate_imu_covariance calculate_imu_covariance.cpp)
ament_target_dependencies(calculate_imu_covariance ${dependencies})

add_executable(test_eigen_pretty_print test_eigen_pretty_print.cpp)
target_link_libraries(test_eigen_pretty_print Eigen3::Eigen)

install(TARGETS
    calculate_imu_covariance
    test_eigen_pretty_print
    DESTINATION lib/${PROJECT_NAME}
)
