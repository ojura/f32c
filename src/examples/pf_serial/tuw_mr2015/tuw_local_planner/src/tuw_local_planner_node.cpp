#include "tuw_local_planner_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>

using namespace tuw;
int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "planner_local" );  /// initializes the ros node with default name
    ros::NodeHandle n;
    LocalPlannerNode planner ( n );
    planner.init();
    ros::Rate rate ( 10 );  /// ros loop frequence synchronized with the wall time (simulated time)

    while ( ros::ok() ) {

        /// calls your loop
        planner.ai();

        /// sets and publishes velocity commands
        planner.publishMotion();

        /// plots measurements
        planner.plot();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

/**
 * Constructor
 **/
LocalPlannerNode::LocalPlannerNode ( ros::NodeHandle & n )
    : LocalPlanner(ros::NodeHandle("~").getNamespace()), 
    n_ ( n ), 
    n_param_ ( "~" ){

    /**
     * @ToDo Wanderer
     * @see http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
     * subscribes the fnc callbackLaser to a topic called "scan"
     * since the simulation is not providing a scan topic /base_scan has to be remaped
     **/
    sub_laser_ = n.subscribe("scan", 1000, &LocalPlannerNode::callbackLaser, this);
    
    /// defines a publisher for velocity commands
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ( "cmd_vel", 1 );

    reconfigureFnc_ = boost::bind ( &LocalPlannerNode::callbackConfigLocalPlanner, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
}

void LocalPlannerNode::callbackConfigLocalPlanner ( tuw_local_planner::LocalPlannerConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigLocalPlanner!" );
    config_ = config;
    init();
}

/**
 * copies incoming laser messages to the base class
 * @param laser
 **/
void LocalPlannerNode::callbackLaser ( const sensor_msgs::LaserScan &_laser ) {
     /**
     * @ToDo Wanderer
     * @see http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
     * creates a callback which fills the measurement_laser_ with the information from the sensor_msgs::LaserScan.
     * to not forget to compute the measurement_laser_[xy].end_point
     **/
    measurement_laser_.range_max() = _laser.range_max;  /// @ToDo
    measurement_laser_.range_min() = _laser.range_min; /// @ToDo
    measurement_laser_.resize ( _laser.ranges.size() ); /// @ToDo
    for ( int i = 0; i < measurement_laser_.size(); i++ ) {
      /// @ToDo
      measurement_laser_ [i].length  = _laser.ranges[i];
      measurement_laser_ [i].angle  = _laser.angle_min + i * _laser.angle_increment;
      measurement_laser_ [i].end_point  = Point2D(0.22+ _laser.ranges[i]*cos( measurement_laser_ [i].angle ),_laser.ranges[i]*sin( measurement_laser_ [i].angle) );
    }
}


/**
 * Publishes motion commands for a robot
 **/
void LocalPlannerNode::publishMotion () {
    geometry_msgs::Twist cmd;
    /// creates motion command
    cmd.linear.x = cmd_.v();
    cmd.linear.y = 0.;
    cmd.angular.z = cmd_.w();
    /// publishes motion command
    pub_cmd_.publish ( cmd );
}
