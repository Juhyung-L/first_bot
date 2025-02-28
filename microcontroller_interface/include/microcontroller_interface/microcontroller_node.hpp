#ifndef MICROCONTROLLER_NODE_HPP_
#define MICROCONTROLLER_NODE_HPP_

#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "msg_2d/msg/odometry_stamped.hpp"
#include "msg_2d/msg/imu_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/utils.h"

#include "libserial/SerialPort.h"

using namespace LibSerial;
using namespace std::placeholders;

union ByteDouble
{
    double val;
    unsigned char bytes[sizeof(double)];
};

union ByteFloat
{
    float val;
    unsigned char bytes[sizeof(float)];
};

template <typename T>
class SerialWrapper
{
public:
    SerialWrapper() = delete;

    SerialWrapper(const std::weak_ptr<rclcpp::Node>& nh, const std::string& port_name, 
        const std::string& topic_name, const std::string& frame_id)
    : nh_(nh)
    , port_name_(port_name)
    , topic_name_(topic_name)
    {
        auto node = nh_.lock();

        pub_ = node->create_publisher<T>(topic_name_, rclcpp::SystemDefaultsQoS());
        logger_ =  node->get_logger();
        clock_ = node->get_clock();
        data_.header.frame_id = frame_id;

        thread_ = std::thread(&SerialWrapper::readLoop, this);
    }

    virtual ~SerialWrapper()
    {
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    SerialPort serial_port_; // make it public to allow concurrent read and write

private:
    void readLoop()
    {
        try
        {
            serial_port_.Open(port_name_.c_str());
        }
        catch (const OpenFailed& e)
        {
            RCLCPP_ERROR(logger_, "%s failed to open. Exiting.", port_name_.c_str());
            return;
        }

        serial_port_.SetBaudRate(BaudRate::BAUD_9600);
        serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        serial_port_.SetParity(Parity::PARITY_NONE);
        serial_port_.SetStopBits(StopBits::STOP_BITS_1);

        while (rclcpp::ok())
        {
            try
            {
                if (!serial_port_ready_)
                {
                    // read until '\n'
                    // blocks until for specified timeout then throws exception
                    std::string line;
                    serial_port_.ReadLine(line, '\n', serial_port_timeout_ms_);

                    // only start processing the lines after the string "Ready" is read
                    if (line.compare("Ready\r\n") == 0)
                    {
                        serial_port_ready_ = true;
                    }   
                    continue;
                }
                readData();
                data_.header.stamp = clock_->now();
                pub_->publish(data_);
            }
            catch (const ReadTimeout& e)
            {
                RCLCPP_ERROR(logger_, 
                    "%s did not read anything for %zu milliseconds. Exiting.",
                    port_name_.c_str(), serial_port_timeout_ms_);
                return;
            }
            catch (const NotOpen& e)
            {
                RCLCPP_ERROR(logger_,
                    "%s is not open. Exiting", port_name_.c_str());
                return;
            }
            catch (std::runtime_error& e)
            {
                RCLCPP_ERROR(logger_, e.what());
                return;
            }
        }

        serial_port_.Close();
    }

protected:
    virtual void readData() = 0;

    std::weak_ptr<rclcpp::Node> nh_;
    std::string port_name_;
    std::string topic_name_;
    std::thread thread_;

    T data_;
    typename rclcpp::Publisher<T>::SharedPtr pub_;
    rclcpp::Logger logger_ = rclcpp::get_logger("tmp_logger");
    rclcpp::Clock::SharedPtr clock_;

    bool serial_port_ready_{false};
    size_t serial_port_timeout_ms_{5000};

    std::function<void()> broadcastTransform{nullptr};
};

class OdomSerial : public SerialWrapper<msg_2d::msg::OdometryStamped>
{
public:
    OdomSerial(const std::weak_ptr<rclcpp::Node>& nh, const std::string& port_name, 
        const std::string& topic_name, const std::string& frame_id)
    : SerialWrapper(nh, port_name, topic_name, frame_id)
    {
        auto node = nh.lock();
        odom_raw_pub_ = node->create_publisher<msg_2d::msg::OdometryStamped>("odom_raw", rclcpp::SystemDefaultsQoS());

        odom_raw_data_.header.frame_id = frame_id;
    }

    ~OdomSerial() override
    {}

protected:
    void readData() override
    {
        // the teensy will send
        // [delta x, delta y, delta yaw, x, y, z]
        // x y z are in world frame
        // delta x y z are in robot frame
        // normally, it should only send delta data, but I am going to compare the accuracy of
        // wheel encoder odometry vs. wheel encoder + IMU EKF fusion
        size_t num_data = 6;
        DataBuffer buffer;
        serial_port_.Read(buffer, num_data*sizeof(double), serial_port_timeout_ms_);

        // convert buffer to vector of doubles
        std::vector<double> doubles(num_data);
        for (size_t i=0; i<num_data; ++i)
        {
            ByteDouble bd;
            for (size_t j=0; j<sizeof(double); ++j)
            {
                bd.bytes[j] = buffer[i*sizeof(double) + j];
            }
            doubles[i] = bd.val;
        }
        
        // delta x y yaw
        data_.x = doubles[0];
        data_.y = doubles[1];
        data_.yaw = doubles[2];

        // x y yaw
        odom_raw_data_.x = doubles[3];
        odom_raw_data_.y = doubles[4];
        odom_raw_data_.yaw = doubles[5];
        odom_raw_data_.header.stamp = clock_->now();
        odom_raw_pub_->publish(odom_raw_data_);
    }

private:
    rclcpp::Publisher<msg_2d::msg::OdometryStamped>::SharedPtr odom_raw_pub_;
    msg_2d::msg::OdometryStamped odom_raw_data_;
};

class IMUSerial : public SerialWrapper<msg_2d::msg::ImuStamped>
{
public:
    IMUSerial(const std::weak_ptr<rclcpp::Node>& nh, const std::string& port_name, 
        const std::string& topic_name, const std::string& frame_id)
    : SerialWrapper(nh, port_name, topic_name, frame_id)
    {}

    ~IMUSerial() override
    {}

protected:
    void readData() override
    {
        // IMU will yaw, ax, and ay
        size_t num_data = 3;
        DataBuffer buffer;
        serial_port_.Read(buffer, num_data*sizeof(float), serial_port_timeout_ms_);

        // convert buffer to vector of floats
        std::vector<float> floats(num_data);
        for (size_t i=0; i<num_data; ++i)
        {
            ByteFloat bf;
            for (size_t j=0; j<sizeof(float); ++j)
            {
                bf.bytes[j] = buffer[i*sizeof(float) + j];
            }
            floats[i] = bf.val;
        }

        data_.yaw = floats[0];
        data_.acc_x = floats[1];
        data_.acc_y = floats[2];
    }
};

class MicrocontrollerNode : public rclcpp::Node
{
public:
    static std::shared_ptr<MicrocontrollerNode> create(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    {
        auto node = std::make_shared<MicrocontrollerNode>(options);
        node->initialize();
        return node;
    }

    MicrocontrollerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("microcontroller_node", options)
    {
        declare_parameter("odom_port", "/dev/ttyACM0");
        declare_parameter("imu_port", "/dev/ttyUSB0");

        declare_parameter("odom_topic", "odom_delta");
        declare_parameter("imu_topic", "imu");
    }

private:
    void initialize()
    {
        std::string odom_port = get_parameter("odom_port").as_string();
        std::string imu_port = get_parameter("imu_port").as_string();

        std::string odom_topic = get_parameter("odom_topic").as_string();
        std::string imu_topic = get_parameter("imu_topic").as_string();

        odom_serial_ = std::make_shared<OdomSerial>(shared_from_this(),
            odom_port, odom_topic, "odom");
        imu_serial_ = std::make_shared<IMUSerial>(shared_from_this(),
            imu_port, imu_topic, "odom");

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&MicrocontrollerNode::cmdVelCB, this, _1));

        using namespace std::chrono_literals;
        timer_ = create_wall_timer(50ms, std::bind(&MicrocontrollerNode::publishCmdVel, this));

        buffer_.resize(3*sizeof(double));
    }

    void cmdVelCB(const geometry_msgs::msg::Twist& cmd_vel)
    {
        std::lock_guard<std::mutex> lock(cmd_vel_mtx_);
        cmd_vel_ = cmd_vel;
    }

    void publishCmdVel()
    {
        std::lock_guard<std::mutex> lock(cmd_vel_mtx_);
        ByteDouble bd;

        // convert 3 doubles into unsigned char and store it in a DataBuffer
        double vel[3] = {cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z};
        for (size_t i=0; i<3; ++i)
        {
            bd.val = vel[i];
            for (size_t j=0; j<sizeof(double); ++j)
            {
                buffer_[i*sizeof(double) + j] = bd.bytes[j];
            }
        }

        try
        {
            odom_serial_->serial_port_.Write(buffer_);
        }
        catch (const NotOpen& e)
        {
            timer_->cancel();
            return;
        }
    }

private:

    std::shared_ptr<OdomSerial> odom_serial_;
    std::shared_ptr<IMUSerial> imu_serial_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex cmd_vel_mtx_;
    geometry_msgs::msg::Twist cmd_vel_;
    // write 3 doubles over serial
    DataBuffer buffer_;
};

#endif
