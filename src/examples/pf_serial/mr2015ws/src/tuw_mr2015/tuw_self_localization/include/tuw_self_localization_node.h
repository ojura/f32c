#ifndef MR_NODE_H
#define MR_NODE_H

#include <memory>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_self_localization/ParticleFilterConfig.h>
#include <tuw_self_localization/KalmanFilterConfig.h>
#include <tuw_self_localization/SelfLocalizationConfig.h>
#include <tuw_self_localization/tuw_self_localization.h>
/**
 * class to cover the ros communication for the self-localization
 **/
class SelfLocalizationNode : public tuw::SelfLocalization {
public:
    SelfLocalizationNode ( ros::NodeHandle & n ); /// Constructor
    void localization();            /// tiggers the self-localization process
    void publishPoseEstimated ();   /// publishes the estimated pose
private:
    ros::NodeHandle n_;             /// node handler to the root node
    ros::NodeHandle n_param_;       /// node handler to the current node
    ros::Subscriber sub_cmd_;       /// Subscriber to the command measurements
    ros::Subscriber sub_odometry_;  /// Subscriber to the odometry measurements
    ros::Subscriber sub_laser_;     /// Subscriber to the laser measurements
    ros::Subscriber sub_initial_pose_; /// Subscriber to receive a standard pose message for initialization (rviz)
    ros::Subscriber sub_ground_truth_; /// Subscriber to the ground truth pose (simulation only)
    ros::Publisher pub_pose_estimated_; /// publisher for the estimated pose
    std::shared_ptr<tf::TransformListener> tf_listener_;  /// listener to receive transformation messages -> to get the laser pose
    geometry_msgs::PoseWithCovarianceStamped pose_; /// pose to publish with covariance
    void callbackCmd ( const geometry_msgs::Twist& ); /// callback function to catch motion commands
    void callbackOdometry ( const nav_msgs::Odometry& ); /// callback function to catch odometry messages
    void callbackGroundTruth ( const nav_msgs::Odometry& ); /// callback function to catch  ground truth pose messages
    void callbackLaser ( const sensor_msgs::LaserScan& ); /// callback function to catch incoming sensor data
    void callbackInitialpose ( const geometry_msgs::PoseWithCovarianceStamped& ); /// callback function to catch init pose messages (rviz)
    
    dynamic_reconfigure::Server<tuw_self_localization::SelfLocalizationConfig> reconfigureServerSelfLocalization_; /// parameter server stuff general use
    dynamic_reconfigure::Server<tuw_self_localization::SelfLocalizationConfig>::CallbackType reconfigureFncSelfLocalization_; /// parameter server stuff general use
    void callbackConfigSelfLocalization ( tuw_self_localization::SelfLocalizationConfig &config, uint32_t level ); /// callback function on incoming parameter changes for general use
    
    std::shared_ptr<dynamic_reconfigure::Server<tuw_self_localization::ParticleFilterConfig> > reconfigureServerParticleFilter_; /// parameter server stuff for the mcl
    dynamic_reconfigure::Server<tuw_self_localization::ParticleFilterConfig>::CallbackType reconfigureFncParticleFilter_; /// parameter server stuff for the mcl
    void callbackConfigParticleFilter ( tuw_self_localization::ParticleFilterConfig &config, uint32_t level ); /// callback function on incoming parameter changes for the mcl
    
    std::shared_ptr<dynamic_reconfigure::Server<tuw_self_localization::KalmanFilterConfig> > reconfigureServerKalmanFilter_; /// parameter server stuff for the ekf
    dynamic_reconfigure::Server<tuw_self_localization::KalmanFilterConfig>::CallbackType reconfigureFncKalmanFilter_; /// parameter server stuff for the ekf
    void callbackConfigKalmanFilter ( tuw_self_localization::KalmanFilterConfig &config, uint32_t level ); /// callback function on incoming parameter changes for the ekf

};

#endif // MR_NOTE_H
