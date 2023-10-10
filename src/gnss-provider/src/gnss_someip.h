#pragma once 

#include <CommonAPI/CommonAPI.hpp>

#include <v0/gnss/GnssServerStubDefault.hpp>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

// #include <chrono>

// #include <v0/gnss/common.hpp>

// using GnssDataMsg = std_msgs::msg::String;
// using GnssData = std::string;

// namespace Types::Conversion {

// /**
//  * @brief converts ROS2 MSG to CommonAPI generated data type 
//  * 
//  * @param gps_data 
//  * @return GnssData 
//  */
// GnssData to_capi_type(const GnssDataMsg & gps_data) {
//     GnssData gnss_data;
//     gnss_data.setStr(gps_data.data);

//     return gnss_data;
// }
// }
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
    static constexpr auto timer_duration = 5s;

    static constexpr auto topic = "GPSD";
    static constexpr auto qos = 10;

public:
    GnssSomeIpReporter()
        : Node(node_name)
        , someip_provider(std::make_shared<T>())
    {
          if(register_someip_service()) {
            RCLCPP_INFO(this->get_logger(), "SOME/IP GnssServer has been registered");

            gpsd_data_subscription = this->create_subscription<std_msgs::msg::String>(topic, qos, std::bind(&GnssSomeIpReporter::on_gpsd_data, this, std::placeholders::_1));

            publish_timer = this->create_wall_timer(timer_duration, [this]() {            
                RCLCPP_INFO(this->get_logger(), "Timer: Broadcast GNSS data over SOME/IP");
        
                std::lock_guard<std::mutex> guard(mutex);

                someip_provider->fireDataEvent(gps_data);
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

    void on_gpsd_data(const std_msgs::msg::String & msg) 
    {
        std::lock_guard<std::mutex> guard(mutex);

        RCLCPP_INFO(this->get_logger(), "Received GPS raw data from GpsdClient node");

        gps_data = msg;
    }

private:
    rclcpp::TimerBase::SharedPtr publish_timer;
    std::shared_ptr<T> someip_provider;

    std::mutex mutex;

    std_msgs::msg::String gps_data;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gpsd_data_subscription;
};