// Builtin ROS2 project
#include "rclcpp/rclcpp.hpp"

// Custom ROS2 project
#include "ros2_input_msgs/msg/xbox_controller_state.hpp"

// Custom library
#include "controller_utilities/xbox/controller.hpp"

// For setting the timer duration
using namespace std::chrono_literals;

class ControllerPublisher : public rclcpp::Node{
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<ros2_input_msgs::msg::XboxControllerState>::SharedPtr publisher;
    Input::Xbox::Controller controller;
    size_t message_count;

public:
    ControllerPublisher() : Node("Controller_Publisher"), message_count(0){
        publisher = this->create_publisher<ros2_input_msgs::msg::XboxControllerState>("topic", 10);
        
        Input::Xbox::detect_controllers();
        if (Input::Xbox::detected_controllers_count() == 0) throw std::runtime_error("Error: No available Xbox controller");
        controller = Input::Xbox::get_controller();
        controller.set_deadzone(0.25f);
        controller.enable_polling();

        auto timer_callback = [this](){
            auto message = ros2_input_msgs::msg::XboxControllerState();
            message.rj_x = controller.rj_x();
            message.rj_y = controller.rj_y();
            message.lj_x = controller.lj_x();
            message.lj_y = controller.lj_y();
            message.rt = controller.rt();
            message.lt = controller.lt();
            message.rj_d = controller.rj_d();
            message.lj_d = controller.lj_d();
            message.lb = controller.lb();
            message.rb = controller.rb();
            message.dl = controller.dpad() == Input::Xbox::DPad::LEFT;
            message.dr = controller.dpad() == Input::Xbox::DPad::RIGHT;
            message.dd = controller.dpad() == Input::Xbox::DPad::DOWN;
            message.du = controller.dpad() == Input::Xbox::DPad::UP;
            message.s = controller.s();
            message.m = controller.m();
            message.h = controller.h();
            message.a = controller.a();
            message.b = controller.b();
            message.x = controller.x();
            message.y = controller.y();
            
            RCLCPP_INFO(
                this->get_logger(), 
                "Publishing (Message Count: %zu):\n\
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
            this->publisher->publish(message);
        };

        // 16ms may be too fast and may over saturate the network in practice
        // For this test, it's fine, though
        timer = this->create_wall_timer(16ms, timer_callback);
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerPublisher>());
    rclcpp::shutdown();
    return 0;
}
