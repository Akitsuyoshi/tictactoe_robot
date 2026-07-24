#!/bin/bash

echo "Deleting grid..."

ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'grid'}"


for i in {0..8}
do
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
    "{name: 'move_$i'}"

    sleep 0.5
done


echo "Deleted"