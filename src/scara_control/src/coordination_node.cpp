#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class CoordinationNode : public rclcpp::Node {
public:
    CoordinationNode() : Node("coordination_node") {
        target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/target_pose", 10,
            std::bind(&CoordinationNode::target_callback, this, std::placeholders::_1));

        joint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/scara/joint_commands", 10);
    }

private:
    void target_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        std_msgs::msg::Float64MultiArray joint_cmds;

        double x = msg->position.x;
        double y = msg->position.y;

        double theta1 = atan2(y, x);
        double theta2 = acos((x*x + y*y - 2*1.0*1.0)/(2*1.0*1.0));

        joint_cmds.data = {theta1, theta2, msg->position.z, 0.0}; // Last value is gripper
        joint_pub_->publish(joint_cmds);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinationNode>());
    rclcpp::shutdown();
    return 0;
}

