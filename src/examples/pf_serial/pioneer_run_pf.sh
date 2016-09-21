#!/bin/bash
case "$1" in
        sedmi_kat)
            PIX_X=900
            PIX_Y=900
            MIN_X=-22.5
            MAX_X=22.5
            MIN_Y=-22.5
            MAX_Y=22.5     
            ;;
        11_kat)
            PIX_X=706
            PIX_Y=706
            MIN_X=-17.65
            MAX_X=17.65
            MIN_Y=-17.65
            MAX_Y=17.65
            ;;
        *)
            echo "Specify environment (sedmi_kat or 11_kat)"
            exit 1
esac


SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

export MAKEFILES=$DIR/../../conf/f32c.mk
export PATH=$PATH:~/gnu_mips_f32c/bin

make 
ujprog ../../../rtl/proj/lattice/ulx2s/sram_pf/impl1/ulx2s_sram_pf_impl1.jed -e pf_serial.bin 

cd mr2015ws
catkin_make

source /opt/ros/kinetic/setup.bash
source devel/setup.bash

export ROS_MASTER_URI=http://charlie.local:11311/
export ROS_HOSTNAME=jura-laptop.local

export MR_DIR=$DIR/mr2015ws/src/tuw_mr2015
rosparam delete /self_localization
rosrun tuw_self_localization tuw_self_localization_node cmd:=/charlie/cmd_vel odom:=/charlie/pose scan:=/charlie/scan _plot_data:=true _mode:=0 _initial_with_ground_truth:=true _plot_data:=true _reinitialize:=false _particle_filter/initial_distribution:=1 _particle_filter/forward_prediction_time:=0 _particle_filter/enable_update:=true _particle_filter/enable_resample:=true _particle_filter/enable_weighting:=true _particle_filter/sigma_init_position:=0.2 _particle_filter/sigma_init_orientation:=0.1 _particle_filter/sigma_static_position:=0.1 _particle_filter/sigma_static_orientation:=0.01 _particle_filter/initial_distribution:=0 _particle_filter/resample_strategy:=0 _map_image:="${MR_DIR}/tuw_self_localization/maps/${1}.png" _map_lines:="${MR_DIR}/tuw_self_localization/maps/cave.yml" _particle_filter/nr_of_samples:=50 _map_pix_x:=$PIX_X _map_pix_y:=$PIX_Y _map_min_x:=$MIN_X _map_max_x:=$MAX_X _map_min_y:=$MIN_Y _map_max_y:=$MAX_Y _particle_filter/sigma_hit:=0.09 _frame_id_base:="charlie/base_link" _frame_id_laser:="charlie/laser" _particle_filter/fpga_map_file:="d:${1}.map" _particle_filter/z_max:=20.0 & #

# to export a map for use on FPGA, add the following argument to tuw_self_localization containing the desired path
# _export_likelihood_mapfile:="/home/juraj/cave.map"

function ctrl_c() {
    #killall roslaunch
    killall tuw_self_localization_node -9
}
trap ctrl_c INT

read
