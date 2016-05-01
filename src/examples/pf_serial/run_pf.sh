#!/bin/bash
source /opt/ros/kinetic/setup.bash

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

cd mr2015ws
catkin_make

source devel/setup.bash
roslaunch tuw_local_planner planner.launch stage:=1 world:=cave mode:=2 &

export MR_DIR=$DIR/mr2015ws/src/tuw_mr2015
rosrun tuw_self_localization tuw_self_localization_node cmd:=cmd_vel scan:=base_scan _plot_data:=true _mode:=0 _initial_with_ground_truth:=true _plot_data:=true _reinitialize:=false _particle_filter/initial_distribution:=1 _particle_filter/forward_prediction_time:=0 _particle_filter/enable_update:=true _particle_filter/enable_resample:=true _particle_filter/enable_weighting:=true _particle_filter/sigma_init_position:=0.2 _particle_filter/sigma_init_orientation:=0.1 _particle_filter/sigma_static_position:=0.05 _particle_filter/sigma_static_orientation:=0.1 _particle_filter/initial_distribution:=0 _particle_filter/resample_strategy:=0 _map_image:="${MR_DIR}/tuw_self_localization/maps/cave.png" _map_lines:="${MR_DIR}/tuw_self_localization/maps/cave.yml" _particle_filter/nr_of_samples:=20
killall roslaunch

#rosrun tuw_self_localization tuw_self_localization_node cmd:=cmd_vel scan:=base_scan _plot_data:=true _mode:=0 _initial_with_ground_truth:=true _plot_data:=true _reinitialize:=false _particle_filter/initial_distribution:=1 _particle_filter/forward_prediction_time:=0 _particle_filter/enable_update:=true _particle_filter/enable_resample:=true _particle_filter/enable_weighting:=true _particle_filter/sigma_init_position:=1 _particle_filter/sigma_init_orientation:=3.14 _particle_filter/sigma_static_position:=0.05 _particle_filter/sigma_static_orientation:=0.1 _particle_filter/initial_distribution:=0 _particle_filter/resample_strategy:=0 _map_image:="/home/juraj/projects/catkin/mr2015ws/src/tuw_mr2015/tuw_self_localization/maps/cave.png" _map_lines:="/home/juraj/projects/catkin/mr2015ws/src/tuw_mr2015/tuw_self_localization/maps/cave.yml" _particle_filter/nr_of_samples:=500
#killall roslaunch
