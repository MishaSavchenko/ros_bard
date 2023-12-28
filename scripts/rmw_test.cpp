#include <iostream>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
    //   publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    //   timer_ = this->create_wall_timer(
    //   500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
    //   auto message = std_msgs::msg::String();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //   publisher_->publish(message);
    }
    // rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // size_t count_;
};

int main(int argc, char * argv[])
{
    std::cout << "wellllll like i know" << std::endl;
    rclcpp::init(argc, argv);
    auto minimal_publisher =  std::make_shared<MinimalPublisher>();
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr  node_graph_interface = minimal_publisher->get_node_graph_interface();


    while(true)
    {
        std::map<std::string, std::vector<std::string>> topics_and_types = node_graph_interface->get_topic_names_and_types();
        for(const auto& elem : topics_and_types)
        {
            std::cout << elem.first << " \n\t";
            for(const auto& e : elem.second)
            {
              std::cout << elem.second[0] << " ";// << elem.second[1] << "\n";  
            } 
            std::cout << std::endl;
        }
        std::cout << "Size of the topic map : " << topics_and_types.size() << std::endl;
        rclcpp::spin_some(minimal_publisher);
        std::cin.get();
    }


    std::cout << "Spinning the minimal publisher node for a while..." << std::endl;
    rclcpp::spin(minimal_publisher);
    rclcpp::shutdown();
    return 0;
}

