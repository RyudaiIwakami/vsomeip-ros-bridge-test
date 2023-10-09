#!/bin/bash

dlt-daemon -p 50002 &
routingmanagerd &

VSOMEIP_CONFIGURATION=/etc/vsomeip-server.json

ros2 run gnss_provider gnss-server

