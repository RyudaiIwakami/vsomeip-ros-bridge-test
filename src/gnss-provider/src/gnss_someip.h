#pragma once 

#include <CommonAPI/CommonAPI.hpp>

#include <v0/gnss/GnssServerStubDefault.hpp>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

#include <types/conversion.h>


#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>



class GnssSomeIpProvider : public v0::gnss::GnssServerStubDefault {

public:
    GnssSomeIpProvider() = default;
    ~GnssSomeIpProvider() = default;
    

    void fireDataEvent(const std_msgs::msg::String & gps_data) {
        
        RCLCPP_INFO(rclcpp::get_logger("GNSS_SOMEIP_Provider"), "Sending gnss data over SOME/IP.");

        auto data = Types::Conversion::to_capi_type(gps_data);

        GnssServerStub::fireDataEvent(data);
    }

};

template <typename T>
class GnssSomeIpReporter : public rclcpp::Node
{
    static constexpr auto node_name = "GNSS_SOMEIP_Reporter";

    static constexpr auto domain = "local";
    static constexpr auto instance = "GnssServer";
    static constexpr auto timer_duration = 50ms;

    static constexpr auto topic = "GPSD";
    static constexpr auto qos = 10;

    // std::chrono::system_clock::time_point  start,end;
    // std::time_t time_stamp;
    
public:
    GnssSomeIpReporter()
        : Node(node_name)
        , someip_provider(std::make_shared<T>())
    {
            
         
          if(register_someip_service()) {
            RCLCPP_INFO(this->get_logger(), "SOME/IP GnssServer has been registered");

            gpsd_data_subscription = this->create_subscription<GnssDataMsg>(topic, qos, std::bind(&GnssSomeIpReporter::on_gpsd_data, this, std::placeholders::_1));

            

            publish_timer = this->create_wall_timer(timer_duration, [this]() {            
                RCLCPP_INFO(this->get_logger(), "Timer: Broadcast GNSS data over SOME/IP");
        
                std::lock_guard<std::mutex> guard(mutex);
                std::ofstream startFile;
                startFile.open("start_times_SOMEIP.txt", std::ios::app);
                
                someip_provider->fireDataEvent(gps_data);
                
                auto start = std::chrono::high_resolution_clock::now();
                auto start_time = std::chrono::time_point_cast<std::chrono::microseconds>(start).time_since_epoch().count();
                startFile << "Run" << std::setw(5) << (time_count_SOMEIP_start++) << ":" << start_time << std::endl;
                // startFile.close(); 
            });
         }  
    }

protected:

    

    bool register_someip_service() {
        if(!CommonAPI::Runtime::get()->registerService(domain,instance, someip_provider)) {
            //TODO: handle error case correctly
            RCLCPP_ERROR(this->get_logger(), "Failed to register SOME/IP GnssServer");
            return false;
        }

        return true;
    }

    void on_gpsd_data(const GnssDataMsg & msg) 
    {

        std::lock_guard<std::mutex> guard(mutex);
        
        // auto end = std::chrono::high_resolution_clock::now();
        // auto end_time = std::chrono::time_point_cast<std::chrono::microseconds>(end).time_since_epoch().count();
        // endFile.open("end_times_ROS2.txt", std::ios::app);
        // endFile << "Run " << std::setw(2) << (time_count_ROS2_end++) << ": " << end_time << " microseconds" << std::endl;
        // // endFile.close();

        RCLCPP_INFO(this->get_logger(), "Received GPS raw data from GpsdClient node");

        gps_data = msg;
    }

private:
    rclcpp::TimerBase::SharedPtr publish_timer;
    std::shared_ptr<T> someip_provider;

    std::mutex mutex;

    std_msgs::msg::String gps_data;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gpsd_data_subscription;

    // int time_count_ROS2_end = 0;
    int time_count_SOMEIP_start = 0;

    
    // std::ofstream endFile;
};