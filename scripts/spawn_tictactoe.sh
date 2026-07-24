#!/bin/bash

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ros2_ws/src/piper_ros/models

cd $HOME/ros2_ws/src/piper_ros/models

BOARD=$1

X=(0.245 0.200 0.155 0.245 0.200 0.155 0.245 0.200 0.155)
Y=(-0.025 -0.025 -0.025 0.020 0.020 0.020 0.065 0.065 0.065 )

echo "Spawning grid..."

ros2 run gazebo_ros spawn_entity.py \
    -entity grid \
    -file grid/model.sdf \
    -x 0.2 \
    -y 0.02 \
    -z 0.02

sleep 1

echo "Spawning pieces..."

for ((i=0;i<9;i++))
do
    CELL=${BOARD:$i:1}

    case "$CELL" in
        X)
            MODEL="cross"
            ;;
        O)
            MODEL="circle"
            ;;
        E)
            continue
            ;;
        *)
            echo "Invalid cell: $CELL"
            continue
            ;;
    esac

    echo "Cell $((i+1)): $MODEL"

    ros2 run gazebo_ros spawn_entity.py \
        -entity move_$i \
        -file $MODEL/model.sdf \
        -x ${X[$i]} \
        -y ${Y[$i]} \
        -z 0.02

    sleep 0.5
done

echo "Finished spawning"