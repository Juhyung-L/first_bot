#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include "msg_2d/msg/pose.hpp"

// this class is solely for visualizing the output of the EKF filter

class Visualizer : public rclcpp::Node
{
public:
    Visualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    :Node("visualizer_node", options)
    {
        declare_parameter("raw_pose_topic", "raw_pose");
        declare_parameter("filtered_pose_topic", "filtered_pose");
        declare_parameter("raw_pose_marker_topic", "raw_pose_marker");
        declare_parameter("filtered_pose_marker_topic", "filtered_pose_marker");

        std::string raw_pose_topic = get_parameter("raw_pose_topic").as_string();
        std::string filtered_pose_topic = get_parameter("filtered_pose_topic").as_string();
        std::string raw_pose_marker_topic = get_parameter("raw_pose_marker_topic").as_string();
        std::string filtered_pose_marker_topic = get_parameter("filtered_pose_marker_topic").as_string();

        raw_pose_sub_ = create_subscription<msg_2d::msg::Pose>(
            raw_pose_topic, rclcpp::SystemDefaultsQoS(), std::bind(&Visualizer::rawPoseCB, this, std::placeholders::_1));
        filtered_pose_sub_ = create_subscription<msg_2d::msg::Pose>(
            filtered_pose_topic, rclcpp::SystemDefaultsQoS(), std::bind(&Visualizer::filteredPoseCB, this, std::placeholders::_1));
        
        raw_pose_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            raw_pose_marker_topic, rclcpp::SystemDefaultsQoS());
        filtered_pose_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            filtered_pose_marker_topic, rclcpp::SystemDefaultsQoS());
    }

private:
    rclcpp::Subscription<msg_2d::msg::Pose>::SharedPtr raw_pose_sub_;
    rclcpp::Subscription<msg_2d::msg::Pose>::SharedPtr filtered_pose_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr raw_pose_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr filtered_pose_marker_pub_;

    int32_t raw_pose_marker_id{0};
    int32_t filtered_pose_marker_id{1};

    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw / 2.0);
        q.w = std::cos(yaw / 2.0);
        return q;
    }

    void setMarker(visualization_msgs::msg::Marker& m, int32_t id)
    {
        m.header.frame_id = "map";
        m.header.stamp = now();
        m.lifetime = rclcpp::Duration::from_seconds(0);
        m.frame_locked = false;
        m.id = id;
        m.color.a = 1.0;
        m.action = visualization_msgs::msg::Marker::ADD;
    }

    void rawPoseCB(const msg_2d::msg::Pose& pose)
    {
        visualization_msgs::msg::Marker m;
        setMarker(m, raw_pose_marker_id);
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; // red
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.scale.x = 0.5; m.scale.y = 0.1; m.scale.z = 0.1;
        m.pose.position.x = pose.x;
        m.pose.position.y = pose.y;
        m.pose.position.z = 0.0;
        m.pose.orientation = yawToQuaternion(pose.yaw);
        raw_pose_marker_pub_->publish(m);
    }

    void filteredPoseCB(const msg_2d::msg::Pose& pose)
    {
        visualization_msgs::msg::Marker m;
        setMarker(m, filtered_pose_marker_id);
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; // green
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.scale.x = 0.5; m.scale.y = 0.1; m.scale.z = 0.1;
        m.pose.position.x = pose.x;
        m.pose.position.y = pose.y;
        m.pose.position.z = 0.0;
        m.pose.orientation = yawToQuaternion(pose.yaw);
        filtered_pose_marker_pub_->publish(m);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Visualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}