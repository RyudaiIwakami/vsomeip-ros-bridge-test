export COMMONAPI_DEFAULT_FOLDER=/home/ros2-humble/vsomeip-ros-bridge/install/gnss_someip_lib/lib/
export COMMONAPI_CONFIG=/home/ros2-humble/vsomeip-ros-bridge/install/gnss_someip_lib/etc/commonapi.ini
source /opt/ros/humble/setup.bash
source /home/ros2-humble/vsomeip-ros-bridge/install/setup.bash

dlt-daemon -p 50002 &

VSOMEIP_APPLICATION_NAME=gnss-client ros2 run gnss_bridge gnss-bridge &

ros2 run gnss_listener gnss-listener



