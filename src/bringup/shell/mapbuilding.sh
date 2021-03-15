#!/bin/bash

#add ros work space path
source /opt/ros/kinetic/setup.bash
source /home/pi/smart_car_ws1/devel/setup.bash

#wait for save map...

#delete old auto_maps, build a new map.
# roscd bringup/build_maps/
# echo "  old maps:"
# ls
# rm -rf auto_map*
# rosrun map_server map_saver -f auto_map
# echo "  your new map:"
# ls

#cd to build_maps, list the old maps
roscd bringup/build_maps/ 
echo "#####" 
echo "--- old maps:"
ls
echo ""

#ask to delet all maps
read -p "--> clear old maps(y/n): " clearmap
if [ $clearmap == "y" ]
then
    read -p "--> clear all maps(y/n): " clearAllmap
    if [ $clearAllmap == "y" ]
    then
        rm -rf ./*
        echo "$ ls"
        ls
    else 
        read -p "--> clear some maps(keyword: name or na): " clearname
        rm -rf ./$clearname*
        echo "$ ls"
        ls 
    fi
fi    
echo ""

#please the user to name the new map,and build it
read -p "--> Enter a name: " name
if [ -z $name ];then 
    echo "  Error!"
    echo ""
    exit
else 
    rm -rf $name*
    rosrun map_server map_saver -f $name
fi

#list the current maps
echo "--- your current maps: "
ls
echo "_____"
