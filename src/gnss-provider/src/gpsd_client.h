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

#include <random>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"

// #include "tf2_ros/static_transform_broadcaster_node.hpp"

// #include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define DATA_SIZE 128 //ファイル名用
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)



class GpsdClient : public rclcpp::Node {
    static constexpr auto node_name = "PC2_Client_node";
    static constexpr auto waiting_time = 1000000;
    static constexpr auto topic = "beforeSOMEIP";
    static constexpr auto qos = 10;
    static constexpr auto gpsd_read_timer_delay = 100ms;

    std::chrono::system_clock::time_point  start;
    // std::time_t time_stamp;

public:


   
    std::ofstream startFile;
    

    GpsdClient(int num_points) : Node(node_name)
    {
        // publisher = this->create_publisher<GnssDataMsg>(topic, qos);
        // if(!init()) {
        //     RCLCPP_WARN(this->get_logger(), "No connection to gpsd");
        // }

        //   if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud_) == -1)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "cannot read pc: %s", pcd_path.c_str());
        //     return;
        // }
        generateRandomPointCloud(num_points);

        // timer = this->create_wall_timer(gpsd_read_timer_delay, std::bind(&GpsdClient::timer_callback, this));
        pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);
        timer_ = this->create_wall_timer(gpsd_read_timer_delay, std::bind(&GpsdClient::timer_callback, this));
  }

    
    ~GpsdClient() = default;
    
    void timer_callback()
    {

        // std::random_device rd;  // 乱数生成器のシードを初期化
        // std::mt19937 gen(rd()); // 標準のメルセンヌ・ツイスター乱数生成器

        // // 位置と回転に使用するための乱数分布を定義（ここでは例として-10.0から10.0まで）
        // std::uniform_real_distribution<> dis(-10.0, 10.0);

        

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud_, msg);
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();

        auto start = std::chrono::high_resolution_clock::now();
        auto start_time = std::chrono::time_point_cast<std::chrono::microseconds>(start).time_since_epoch().count();

        pcd_publisher_->publish(msg);

        
        startFile.open("PointCloud2_evaluation/evaluation_" TOSTRING(DATA_SIZE) "/01_start_times_ROS2_" TOSTRING(DATA_SIZE) ".csv", std::ios::app);
        startFile << "Run " << std::setw(6) << (time_count_ROS2_start++) << ":" << start_time << std::endl;
        startFile.close();
        //タイムスタンプをファイルに書き込む
        RCLCPP_INFO(this->get_logger(), "%d\n",count++);
    }

private:

 void generateRandomPointCloud(int num_points)
    {
        cloud_->width = num_points;
        cloud_->height = 1;
        cloud_->points.resize(cloud_->width * cloud_->height);

        for (auto &point : cloud_->points)
        {
            point.x = 1024 * rand() / (RAND_MAX + 1.0f);
            point.y = 1024 * rand() / (RAND_MAX + 1.0f);
            point.z = 1024 * rand() / (RAND_MAX + 1.0f);
        }
    }

    std::mutex mutex;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<GnssDataMsg>::SharedPtr publisher;
    size_t count_;
    int time_count_ROS2_start = 0;
    int time_count_all_start = 0;
    int count = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    
};