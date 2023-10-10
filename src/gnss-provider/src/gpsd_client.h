// #pragma once 

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/serialization.hpp>



// #include <libgpsmm.h>

// namespace Types::Conversion {

// /**
//  * @brief converts gpsd datatype gps_data_t to ros2 msg generated type
//  * 
//  * @param gps_data 
//  * @return GnssDataMsg 
//  */
// GnssDataMsg to_gnss_data_msg(const gps_data_t * gps_data) {
//     GnssDataMsg msg; 

//     msg.position.fix.latitude = gps_data->fix.latitude;
//     msg.position.fix.longitude = gps_data->fix.longitude;
//     msg.position.dop.hdop = gps_data->dop.hdop;
//     msg.position.dop.vdop = gps_data->dop.vdop;
//     msg.position.satellites_visible = gps_data->satellites_visible;
//     msg.position.satellites_used = gps_data->satellites_used;

//     return msg;
// }

// } // namespace Types::Conversion

// // inspired by https://gist.github.com/ncoder-1/8313815ac387e6757f751dc8960f03d7
// class GpsdClient : public rclcpp::Node {

//     static constexpr auto node_name = "GPSD_Client_node";

//     static constexpr auto gpsd_host = "localhost";
//     static constexpr auto waiting_time = 10000000;

//     static constexpr auto topic = "GPSD";
//     static constexpr auto qos = 10;

//     static constexpr auto gpsd_read_timer_delay = 500ms;

// public:
//     GpsdClient()
//         : Node(node_name),
//         gps_rec(gpsd_host, "50001") 
//     {
//         publisher = this->create_publisher<GnssDataMsg>(topic, qos);

//         if(!init()) {
//             RCLCPP_WARN(this->get_logger(), "No connection to gpsd");
//         }

//         timer = this->create_wall_timer(gpsd_read_timer_delay, std::bind(&GpsdClient::read, this));

//         //TODO: handle case when there is no connection to gpsd#include <rclcpp/serialization.hpp>
//             return false;
//         }        

//         return true;
//     }

//     void read() {
//         for(;;) 
//         {
//             std::lock_guard<std::mutex> lock_guard(mutex);
            
//             if (!gps_rec.waiting(waiting_time)) {
//                 RCLCPP_WARN(this->get_logger(), "Waiting, failed to read data from gpsd");
//                 continue;
//             }

//             struct gps_data_t* gpsd_data;

//             if ((gpsd_data = gps_rec.read()) == nullptr) {
//                 RCLCPP_WARN(this->get_logger(), "Null, Failed to read data from gpsd");
//                 continue;
//             }

//             RCLCPP_WARN(this->get_logger(), "Obtained data from gpsd");

//             auto msg = Types::Conversion::to_gnss_data_msg(gpsd_data);
//             msg.position.fix.latitude = 01231.2946;
//             RCLCPP_WARN(this->get_logger(),"Message: '%f'",msg.position.fix.latitude);
//             publisher->publish(msg);

//             return;
//         }
//     }

// protected:

// private:
//     gpsmm gps_rec;

//     std::mutex mutex;

//     rclcpp::TimerBase::SharedPtr timer;
//     rclcpp::Publisher<GnssDataMsg>::SharedPtr publisher;
// };


#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <fstream>
#include <string>
#include <vector>


#include "std_msgs/msg/string.hpp"

using GnssDataMsg = std_msgs::msg::String;
using GnssData = std::string;

// namespace Types::Conversion {
// GnssDataMsg to_gnss_data_msg(const std_msgs::msg::String * gps_data) {
//     GnssDataMsg msg;
//     msg.data = gps_data->data;
//     return msg;
// }
// } // namespace Types::Conversion
class GpsdClient : public rclcpp::Node {
    static constexpr auto node_name = "GPSD_Client_node";
    static constexpr auto waiting_time = 10000000;
    static constexpr auto topic = "GPSD";
    static constexpr auto qos = 10;
    static constexpr auto gpsd_read_timer_delay = 500ms;
public:
    GpsdClient() : Node(node_name)
    {
        publisher = this->create_publisher<std_msgs::msg::String>(topic, qos);
        // if(!init()) {
        //     RCLCPP_WARN(this->get_logger(), "No connection to gpsd");
        // }

        timer = this->create_wall_timer(gpsd_read_timer_delay, std::bind(&GpsdClient::timer_callback, this));
        // read_nmea_file("/home/ros2-humble/ros2_gps_publisher/gnss-sentences.nmea");
    }
    ~GpsdClient() = default;
    // bool init() {
    //     if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == nullptr) {
    //         return false;
    //     }
    //     return true;
    // }
    // void read_nmea_file(const std::string &path) {
    //     std::ifstream nmea_file(path);
    //     if (!nmea_file.is_open()) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to open NMEA file");
    //         return;
    //     }
    //     std::string line;
    //     while (std::getline(nmea_file, line)) {
    //         nmea_lines.push_back(line);
    //     }
    // }
    // void read() {
    //     if (index >= nmea_lines.size()) {
    //         return;
    //     }
    //     std::string line = nmea_lines[index];
    //     index++;
    //     if (line.find("$GPGGA") != std::string::npos) {
    //         try {
    //             auto msg = GnssDataMsg();
    //             msg.position.fix.latitude = 0.0;  // Replace with actual latitude parsing
    //             msg.position.fix.longitude = 0.0; // Replace with actual longitude parsing
    //             publisher->publish(msg);
    //         }
    //         catch (const std::exception &e) {
    //             RCLCPP_WARN(this->get_logger(), "Parse error in NMEA string");
    //         }
    //     }
    // }
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher->publish(message);
    }

    

private:

    std::mutex mutex;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    size_t count_;
};