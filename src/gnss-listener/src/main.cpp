#include "rclcpp/rclcpp.hpp"

// #include <gnss_someip_lib/msg/gnss_data.hpp>

#include "std_msgs/msg/string.hpp"



#include <sensor_msgs/msg/point_cloud2.hpp>




using  GpsDataMsg = sensor_msgs::msg::PointCloud2;
class GnssTopicSubsriber : public rclcpp::Node
{
  static constexpr auto node_name = "PC2_Topic_Subscriber";

  static constexpr auto topic = "afterSOMEIP";
  static constexpr auto qos = 10;

public:
    GnssTopicSubsriber() : Node(node_name)
    {
      subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic, qos, std::bind(&GnssTopicSubsriber::topic_callback, this, std::placeholders::_1));
    }

private:
    // void gnss_topic_callback(const GpsDataMsg & msg) const
    // {
    //   RCLCPP_INFO(this->get_logger(), "GNSS position latitude %f, longitude %f", 
    //       msg.position.fix.latitude,
    //       msg.position.fix.longitude
    //     );
    // }

     void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
     RCLCPP_INFO(rclcpp::get_logger("PointCloud2Logger_topic"), 
        "PointCloud2 Info:\n"
        " - Height: %u\n"
        " - Width: %u\n"
        " - Point Step: %u\n"
        " - Row Step: %u\n"
        " - Is Dense: %s\n"
        " - Data Size: %lu",
        msg->height,
        msg->width,
        msg->point_step,
        msg->row_step,
        msg->is_dense ? "True" : "False",
        msg->data.size()
        
        );

         std::stringstream tt;
                    for (const auto& field : msg->fields) {
                    tt << "Name: " << field.name<< ", "
                    << "Offset: " << field.offset << ", "
                    << "Datatype: " << static_cast<int>(field.datatype) << ", "
                    << "Count: " << field.count << std::endl;
                }
                    RCLCPP_INFO(rclcpp::get_logger("logger_name"), "PointCloud2 Fields LI:\n%s", tt.str().c_str());


  }
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssTopicSubsriber>());
  rclcpp::shutdown();
  return 0;
}
