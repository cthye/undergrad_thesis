#!/bin/bash

#add ros work space path
source /opt/ros/kinetic/setup.bash
source /home/pi/smart_car_ws/devel/setup.bash

#wait for save map...

#delete old auto_maps, build a new map.
roscd bringup/build_maps/
echo "  old maps:"
ls
rm -rf auto_map*
rosrun map_server map_saver -f auto_map
echo "  your new map:"
ls

