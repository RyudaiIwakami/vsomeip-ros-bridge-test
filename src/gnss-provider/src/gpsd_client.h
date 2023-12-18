#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

// #include "geometry_msgs/msg/pose.hpp"

#include <string>
#include <vector>


#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/static_transform_broadcaster_node.hpp"

#include "tf2_ros/transform_broadcaster.h"


class GpsdClient : public rclcpp::Node {
    static constexpr auto node_name = "STR_Client_node";
    static constexpr auto waiting_time = 1000000;
    static constexpr auto topic = "beforeSOMEIP";
    static constexpr auto qos = 10;
    static constexpr auto gpsd_read_timer_delay = 2000ms;

    // std::chrono::system_clock::time_point  start;
    // std::time_t time_stamp;

public:

    

    GpsdClient() : Node(node_name)
    {
        // publisher = this->create_publisher<GnssDataMsg>(topic, qos);
        // if(!init()) {
        //     RCLCPP_WARN(this->get_logger(), "No connection to gpsd");
        // }

        timer = this->create_wall_timer(gpsd_read_timer_delay, std::bind(&GpsdClient::timer_callback, this));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

    
    ~GpsdClient() = default;
    
    void timer_callback()
    {

   geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "parent_frame";
    t.child_frame_id = "child_frame";
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 3.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(), "Sent transform: [%f, %f, %f]",
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z);

    // RCLCPP_INFO(this->get_logger(), "Publishing position x '%f'", message.position.x);

    //   publisher->publish(message);
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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    
};