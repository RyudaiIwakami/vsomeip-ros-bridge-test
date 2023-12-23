#pragma once

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/GnssServerProxy.hpp>
#include "std_msgs/msg/string.hpp"

#include <types/conversion.h>
#include <optional>


#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>



// #include <iostream>


using GpsDataMsg = sensor_msgs::msg::PointCloud2;
using GnssData = v0::gnss::common::PointCloud2;
// std::ofstream endFile;

template<template<typename ...> class P>
class AbstractSomeIpClient 
{
    using ProxyClass = P<>;
    using ProxyClassPtr = std::shared_ptr<ProxyClass>;
    
    

public: 
    AbstractSomeIpClient(std::string domain, std::string instance) 
        : proxy_(CommonAPI::Runtime::get()->buildProxy<P>(domain,instance)) 
    {
        init();
    }


    virtual std::optional<bool> available() {
        return (initialised()) ? std::make_optional(proxy_->isAvailable()) : std::nullopt;
    }

    virtual void onAvailable() = 0;

protected:
    ProxyClassPtr proxy() {
        return proxy_;
    }

    virtual bool initialised() {
        return proxy_ != nullptr; 
    }

    void init() {
        if(!proxy_)
        {
            // std::cout << "Hello, World!" << std::endl;

            // RCLCPP_ERROR(this->get_logger(), "Not able to initialize SOME/IP proxy for GNSS");
            //TODO: handle error case correctly
            return;
        }

        proxy_->getProxyStatusEvent().subscribe(std::bind(&AbstractSomeIpClient<P>::onAvailablilityStatusChange, this, std::placeholders::_1));
    }

    virtual void onAvailablilityStatusChange(CommonAPI::AvailabilityStatus status) {
        // first event is always CommonAPI::AvailabilityStatus::NOT_AVAILABLE

        if (status == CommonAPI::AvailabilityStatus::AVAILABLE)
        {
            onAvailable();
        }
    }

private:
    ProxyClassPtr proxy_;
};



using GnssSomeIpProxyWrapper = AbstractSomeIpClient<v0::gnss::GnssServerProxy>;

class GnssSomeIpClient : public GnssSomeIpProxyWrapper
{
    static constexpr auto domain = "local";
    static constexpr auto instance = "GnssServer";


    // std::ofstream endFile;

    using MessageCallback = std::function<void(const GpsDataMsg & message)>;
    

public:
    GnssSomeIpClient() : GnssSomeIpProxyWrapper(domain, instance) {}

    
    void setMessageCallback(MessageCallback callback) {
        message_callback = std::move(callback);
    }

    

    

    void onAvailable() override {

            
            proxy()->getDataEvent().subscribe([this](const GnssData & data) {

                    std::stringstream ss;
                    for (const auto& field : data.getFields()) {
                    ss << "Name: " << field.getName() << ", "
                    << "Offset: " << field.getOffset() << ", "
                    << "Datatype: " << static_cast<int>(field.getDatatype()) << ", "
                    << "Count: " << field.getCount() << std::endl;
                }
                    RCLCPP_INFO(rclcpp::get_logger("logger_name"), "PointCloud2 Fields before convert:\n%s", ss.str().c_str());
            

            auto message = Types::Conversion::from_capi_type(data);

            std::stringstream tt;
                    for (const auto& field : message.fields) {
                    tt << "Name: " << field.name<< ", " 
                    << "Offset: " << field.offset << ", "
                    << "Datatype: " << static_cast<int>(field.datatype) << ", "
                    << "Count: " << field.count << std::endl;

                }
                    RCLCPP_INFO(rclcpp::get_logger("logger_name"), "PointCloud2 Fields after convert BR:\n%s", tt.str().c_str());

            // auto end = std::chrono::high_resolution_clock::now();
            // auto end_time = std::chrono::time_point_cast<std::chrono::microseconds>(end).time_since_epoch().count();
            
            
            // std::ofstream endFile;
            // endFile.open("end_times_SOMEIP.csv", std::ios::app);
            // endFile << "Run" << std::setw(5) << (time_count_SOMEIP_end++) << ":" << end_time << std::endl;
            
            message_callback(message);
        });
    }

private:
    MessageCallback message_callback;
    int time_count_SOMEIP_end = 0;
    
};
