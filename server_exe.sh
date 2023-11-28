export COMMONAPI_DEFAULT_FOLDER=/home/ros2-humble/vsomeip-ros-bridge/install/gnss_someip_lib/lib/
export COMMONAPI_CONFIG=/home/ros2-humble/vsomeip-ros-bridge/install/gnss_someip_lib/etc/commonapi.ini
source /opt/ros/humble/setup.bash
source /home/ros2-humble/vsomeip-ros-bridge/install/setup.bash

dlt-daemon &
routingmanagerd &

VSOMEIP_APPLICATION_NAME=gnss-server ros2 run gnss_provider gnss-server


