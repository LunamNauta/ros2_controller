// Builtin ROS2 project
#include "rclcpp/rclcpp.hpp"

// Custom ROS2 project
#include "ros2_input_msgs/msg/xbox_controller_state.hpp"

// Custom library
//#include "controller_utilities/xbox/controller.hpp"

class ControllerSubscriber : public rclcpp::Node{
private:
    rclcpp::Subscription<ros2_input_msgs::msg::XboxControllerState>::SharedPtr subscription;
    std::size_t message_count;

public:
    ControllerSubscriber() : Node("Controller_Subscriber"), message_count(0){
        auto topic_callback = [this](const ros2_input_msgs::msg::XboxControllerState& message){
            RCLCPP_INFO(
                this->get_logger(), 
                "I heard (Message Count: %zu):\n\
                RJ=(%.7f, %.7f) LJ=(%.7f, %.7f)\n\
                RT=%.7f RB=%.7f\n\
                RJ_D=%i LJ_D=%i\n\
                RB=%i   LB=%i\n\
                DL=%i   DR=%i  DD=%i  DU=%i\n\
                S=%i    M=%i   H=%i\n\
                A=%i    B=%i   X=%i  Y=%i\n\n",
                message_count++,
                message.rj_x, message.rj_y, message.lj_x, message.lj_y,
                message.rt, message.lt,
                message.rj_d, message.lj_d,
                message.rb, message.lb,
                message.dl, message.dr, message.dd, message.du,
                message.s, message.m, message.h,
                message.a, message.b, message.x, message.y
            );
        };
        subscription = this->create_subscription<ros2_input_msgs::msg::XboxControllerState>("topic", 10, topic_callback);
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerSubscriber>());
    rclcpp::shutdown();
    return 0;
}
