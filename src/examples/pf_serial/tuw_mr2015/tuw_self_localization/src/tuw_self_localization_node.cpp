#include "tuw_self_localization_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>
#include <boost/filesystem.hpp>
#include <tf/transform_listener.h>

using namespace tuw;

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "self_localization" );
    ros::NodeHandle n;
    SelfLocalizationNode self_localization ( n );
    self_localization.init();
    ros::Rate rate ( 10 );

    while ( ros::ok() ) {

        /// localization
        self_localization.localization();

        /// publishes the estimated pose
        self_localization.publishPoseEstimated();

        /// plots measurments
        self_localization.plot();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleep for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

/**
 * Constructor
 **/
SelfLocalizationNode::SelfLocalizationNode ( ros::NodeHandle & n )
    : SelfLocalization ( ros::NodeHandle ( "~" ).getNamespace() ),
      n_ ( n ),
      n_param_ ( "~" ) {

    // reads shared parameter on the operation mode
    int mode;
    n_param_.getParam ( "mode", mode );
    if ( mode == PoseFilter::PARTICLE_FILTER ) pose_filter_ = std::make_shared<tuw::ParticleFilter>();
    if ( mode == PoseFilter::KALMAN_FILTER )   pose_filter_ = std::make_shared<tuw::KalmanFilter>();

    ROS_INFO ( "mode: %s(%i)", pose_filter_->getTypeName().c_str(), ( int ) pose_filter_->getType() );
    n_param_.getParam ( "map_image", filename_map_image_ );

    if ( boost::filesystem::exists ( filename_map_image_ ) )  {
        ROS_INFO ( "map_image: %s", filename_map_image_.c_str() );
    } else {
        ROS_ERROR ( "map_image file does not exist: %s", filename_map_image_.c_str() );
        return;
    }
    n_param_.getParam ( "map_lines", filename_map_lines_ );

    if ( boost::filesystem::exists ( filename_map_lines_ ) )  {
        ROS_INFO ( "map_lines: %s", filename_map_lines_.c_str() );
    } else {
        ROS_ERROR ( "map_lines file does not exist: %s", filename_map_lines_.c_str() );
        return;
    }
    /// subscribes to transforamtions
    tf_listener_ = std::make_shared<tf::TransformListener>();

    n_param_.param<std::string> ( "frame_id_map", pose_.header.frame_id, "map" );

    /// subscribes to  odometry values
    sub_cmd_ = n.subscribe ( "cmd", 1, &SelfLocalizationNode::callbackCmd, this );

    /// subscribes to  odometry values
    sub_odometry_ = n.subscribe ( "odom", 1, &SelfLocalizationNode::callbackOdometry, this );

    /// two subscribers to laser sensor
    sub_initial_pose_ = n.subscribe ( "initialpose", 1, &SelfLocalizationNode::callbackInitialpose, this );

    /// two subscribers to ground truth data
    sub_ground_truth_ = n.subscribe ( "base_pose_ground_truth", 1, &SelfLocalizationNode::callbackGroundTruth, this );

    /// defines a publisher for the resulting pose
    pub_pose_estimated_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "pose_estimated", 1 );

    pose_.header.seq = 0;

    /// start parameter server
    reconfigureFncSelfLocalization_ = boost::bind ( &SelfLocalizationNode::callbackConfigSelfLocalization, this,  _1, _2 );
    reconfigureServerSelfLocalization_.setCallback ( reconfigureFncSelfLocalization_ );

    /// two subscribers to laser sensor
    sub_laser_ = n.subscribe ( "scan", 1, &SelfLocalizationNode::callbackLaser, this );

    if ( pose_filter_->getType() == PoseFilter::PARTICLE_FILTER ) {

        /// start parameter server
        reconfigureServerParticleFilter_ = std::make_shared< dynamic_reconfigure::Server<tuw_self_localization::ParticleFilterConfig> > ( ros::NodeHandle ( "~/particle_filter" ) );
        reconfigureFncParticleFilter_ = boost::bind ( &SelfLocalizationNode::callbackConfigParticleFilter, this,  _1, _2 );
        reconfigureServerParticleFilter_->setCallback ( reconfigureFncParticleFilter_ );
    }
    if ( pose_filter_->getType() == PoseFilter::KALMAN_FILTER ) {

        /// start parameter server
        reconfigureServerKalmanFilter_ = std::make_shared< dynamic_reconfigure::Server<tuw_self_localization::KalmanFilterConfig> > ( ros::NodeHandle ( "~/kalman_filter" ) );
        reconfigureFncKalmanFilter_ = boost::bind ( &SelfLocalizationNode::callbackConfigKalmanFilter, this,  _1, _2 );
        reconfigureServerKalmanFilter_->setCallback ( reconfigureFncKalmanFilter_ );
    }
}


void SelfLocalizationNode::callbackConfigSelfLocalization ( tuw_self_localization::SelfLocalizationConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigSelfLocalization!" );
    config_ = config;
    init();
}

void SelfLocalizationNode::callbackConfigParticleFilter ( tuw_self_localization::ParticleFilterConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigParticleFilter!" );
    pose_filter_->setConfig ( &config );
}

void SelfLocalizationNode::callbackConfigKalmanFilter ( tuw_self_localization::KalmanFilterConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigKalmanFilter!" );
    pose_filter_->setConfig ( &config );
}

void SelfLocalizationNode::localization() {
    if ( config_.reinitialize ) {
        pose_filter_->setPoseInit ( pose_ground_truth_ );
        pose_filter_->reset();
    }
    /// localization
    SelfLocalization::localization ();
}



/**
 * copies incoming laser messages to the base class
 * @param laser
 **/
void SelfLocalizationNode::callbackLaser ( const sensor_msgs::LaserScan &_laser ) {
    tf::StampedTransform transform;
    /**
     * @ToDo MotionModel
     * remove the static transforamtion and use the tf_listener_
     * @url http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
     **/ 
   //measurement_laser_->tf() = Pose2D ( 0.225,  0, 0).tf();  /// to remove
    
    
    
    try{
      tf::TransformListener listener;
      listener.waitForTransform("/base_link", "/base_laser_link", ros::Time(0), ros::Duration(4.0));
      listener.lookupTransform("/base_link", "/base_laser_link", ros::Time(0), transform);
       }
       catch (tf::TransformException &ex) {
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
       }
    
    
    double yaw,pitch,roll;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw,pitch,roll);
    
    //printf("%f %f %lf\n", transform.getOrigin().x(), transform.getOrigin().y(), yaw);
    measurement_laser_->tf() = Pose2D ( transform.getOrigin().x(), transform.getOrigin().y(), yaw ).tf();
    
    /// your code
    
    int nr = ( _laser.angle_max - _laser.angle_min ) / _laser.angle_increment;
    measurement_laser_->range_max() = _laser.range_max;
    measurement_laser_->range_min() = _laser.range_min;
    measurement_laser_->resize ( nr );
    measurement_laser_->stamp() = _laser.header.stamp.toBoost();
    for ( int i = 0; i < nr; i++ ) {
        MeasurementLaser::Beam &beam = measurement_laser_->operator[] ( i );
        beam.length = _laser.ranges[i];
        beam.angle = _laser.angle_min + ( _laser.angle_increment * i );
        beam.end_point.x() = cos ( beam.angle ) * beam.length;
        beam.end_point.y() = sin ( beam.angle ) * beam.length;
    }
}

/**
 * copies incoming odemetry messages to the base class
 * @param odom
 **/
void SelfLocalizationNode::callbackOdometry ( const nav_msgs::Odometry &odom ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( odom.pose.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    odom_.set ( odom.pose.pose.position.x, odom.pose.pose.position.y, a );
}

/**
 * copies incoming robot pose messages to the base class
 * @param pose
 **/
void SelfLocalizationNode::callbackInitialpose ( const geometry_msgs::PoseWithCovarianceStamped &pose ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( pose.pose.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    pose_filter_->reinitialize ( Pose2D ( pose.pose.pose.position.x, pose.pose.pose.position.y, a ) );
}


/**
 * copies incoming robot command message
 * @param cmd
 **/
void SelfLocalizationNode::callbackCmd ( const geometry_msgs::Twist& cmd ) {
    cmd_.v() = cmd.linear.x;
    cmd_.w() = cmd.angular.z;
}

/**
 * copies incoming odemetry messages to the base class
 * @param odom
 **/
void SelfLocalizationNode::callbackGroundTruth ( const nav_msgs::Odometry& ground_truth ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( ground_truth.pose.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    pose_ground_truth_.set ( ground_truth.pose.pose.position.x, ground_truth.pose.pose.position.y, a );

    if ( config_.initial_with_ground_truth ) {
        ROS_INFO ( "initial_with_ground_truth!" );
        pose_filter_->reinitialize ( pose_ground_truth_ );
        config_.initial_with_ground_truth = false;
    }
}

/**
 * Publishes the estimated pose
 **/
void SelfLocalizationNode::publishPoseEstimated () {
    pose_.header.stamp.fromBoost ( pose_filter_->time_last_update() );
    pose_.header.seq++;
    pose_.pose.pose.position.x = pose_estimated_.x();
    pose_.pose.pose.position.y = pose_estimated_.y();
    pose_.pose.pose.position.z = 0;
    pose_.pose.pose.orientation = tf::createQuaternionMsgFromYaw ( pose_estimated_.theta() );
    for ( double &d : pose_.pose.covariance ) d = 0;
    /// publishes motion command
    pub_pose_estimated_.publish ( pose_ );
}
