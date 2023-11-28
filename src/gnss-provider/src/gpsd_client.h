#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <string>
#include <vector>


#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>


#include "std_msgs/msg/string.hpp"

class GpsdClient : public rclcpp::Node {
    static constexpr auto node_name = "STR_Client_node";
    static constexpr auto waiting_time = 1000000;
    static constexpr auto topic = "beforeSOMEIP";
    static constexpr auto qos = 10;
    static constexpr auto gpsd_read_timer_delay = 500ms;

    // std::chrono::system_clock::time_point  start;
    // std::time_t time_stamp;

public:

    

    GpsdClient() : Node(node_name)
    {
        publisher = this->create_publisher<GnssDataMsg>(topic, qos);
        // if(!init()) {
        //     RCLCPP_WARN(this->get_logger(), "No connection to gpsd");
        // }

        timer = this->create_wall_timer(gpsd_read_timer_delay, std::bind(&GpsdClient::timer_callback, this));
    }
    ~GpsdClient() = default;
    
    void timer_callback()
    {
    //   std::ofstream startFile;
    //   startFile.open("start_times_ROS2_byte.csv", std::ios::app);
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

      publisher->publish(message);
    //   auto start = std::chrono::high_resolution_clock::now();
    //   auto start_time = std::chrono::time_point_cast<std::chrono::microseconds>(start).time_since_epoch().count();
    //   startFile << "Run" << std::setw(5) << (time_count_ROS2_start++) << ":" << start_time << std::endl;
    }

private:

    std::mutex mutex;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<GnssDataMsg>::SharedPtr publisher;
    size_t count_;
    int time_count_ROS2_start = 0;
    
};