#include "tuw_local_planner/tuw_local_planner.h"
#include <opencv/cv.h>
#include <boost/concept_check.hpp>
#include <cstdlib>

using namespace cv;
using namespace tuw;

std::map<LocalPlanner::ControlMode, std::string> LocalPlanner::ControlModeName_ = {
    {DEMO, "DEMO"},
    {STOP, "STOP"},
    {WANDERER, "WANDERER"},
    {WANDERER2, "WANDERER2"},
    {WALL_FOLLOWING, "WALL_FOLLOWING"},
    {WALL_FOLLOWING2, "WALL_FOLLOWING2"},
    {GOTO, "GOTO"},
    {GOTO2, "GOTO2"}
};

LocalPlanner::LocalPlanner ( const std::string &ns )
    : loop_count_ ( 0 )
    , figure_local_ ( ns + ", Local View" ) {

}

void LocalPlanner::init() {

    figure_local_.init ( config_.map_pix_x, config_.map_pix_y,
                         config_.map_min_x, config_.map_max_x,
                         config_.map_min_y, config_.map_max_y,
                         config_.map_rotation + M_PI/2.0,
                         config_.map_grid_x, config_.map_grid_y );

    cv::putText ( figure_local_.background(), ControlModeName_[ ( ControlMode ) config_.mode],
                  cv::Point ( 5,figure_local_.view().rows-10 ),
                  cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, CV_AA );

    /**
     * @ToDo Wanderer
     * changes the Maxima Musterfrau to your name
     **/
    cv::putText ( figure_local_.background(), "Juraj Orsulic",
                  cv::Point ( figure_local_.view().cols-250, figure_local_.view().rows-10 ),
                  cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, CV_AA );
}

void LocalPlanner::plot() {
    if ( config_.plot_data ) plotLocal();
    cv::waitKey ( 10 );
}

void LocalPlanner::plotLocal() {
    figure_local_.clear();
    for ( size_t i = 0; i < measurement_laser_.size(); i++ ) {
        /**
        * @ToDo Wanderer
        * uses Figure::symbol or Figure::circle to plot the laser measurements in a for loop
        **/
	
	figure_local_.circle ( measurement_laser_[i].end_point, 1, Figure::red );

    }
    cv::imshow ( figure_local_.title(),figure_local_.view() );
}

void LocalPlanner::ai() {
    if ( measurement_laser_.empty() ) {
        cmd_.set ( 0, 0 );
        return;
    }
    switch ( config_.mode ) {
    case STOP:
        cmd_.set ( 0, 0 );
        break;
    case DEMO:
        ai_demo();
        break;
    case WANDERER:
        ai_wanderer();
        break;
    case WANDERER2:
        ai_wanderer2();
        break;
    default:
        cmd_.set ( 0, 0 );
    }
    loop_count_++;
}

/**
* Demo
**/
void LocalPlanner::ai_demo() {
    double v = 0.0, w = 0.0;
    if ( measurement_laser_.empty() ) {
        v = 0.2, w = -0.02;
    } else {
        if ( measurement_laser_[measurement_laser_.size() / 4].length < 1.0 ) {
            w = 0.4;
        } else {
            v = 0.4;
        }
    }
    cmd_.set ( v, w );
}
    #define pi 3.14159
/**
* @ToDo Wanderer
* writes one or two wanderer behaviours to keep the robot at least 120sec driving without a crash by exploring as much as possible.
* I know it sounds weird but don't get too fancy. 
**/
void LocalPlanner::ai_wanderer() {
    
    double danger_left=0, danger_right=0;
    int i; double min_dist = measurement_laser_[0].length; double max_dist=0; double min_angle = measurement_laser_[0].angle; double max_angle = min_angle;
    for(i=0; i< measurement_laser_.size(); i++) {  
      
	// find the direction with largest free distance
      	if(measurement_laser_[i].length > max_dist) { max_dist = measurement_laser_[i].length;
	  max_angle =  measurement_laser_[i].angle; }
	    
      // check for immediate danger on right/left
      if(measurement_laser_[i].length < 0.5 && measurement_laser_[i].angle > 0 && measurement_laser_[i].angle < pi/3) danger_left = -0.5;
      if(measurement_laser_[i].length < 0.5 && measurement_laser_[i].angle < 0 && measurement_laser_[i].angle > -pi/3) danger_right = 0.5;
    }

    // steer to direction with largest distance; some hysteresis allowed before steering from current direction
    if(danger_left == 0 && max_angle > pi/30 && max_dist > measurement_laser_[measurement_laser_.size()/2].length + 0.5) cmd_.set(0.1, 0.5);
    else if(danger_right == 0 && max_angle < -pi/30 && max_dist > measurement_laser_[measurement_laser_.size()/2].length + 0.5 ) cmd_.set(0.1, -0.5);
    // straight line mode (hysteresis active), plus collision avoidance
    else cmd_.set(0.7 - fabs(danger_left + danger_right),danger_left+danger_right);

    
}
/**
* @ToDo Wanderer
* OPTIONAL: if you like you can write a other one :-)
**/
void LocalPlanner::ai_wanderer2() {
    cmd_.set ( 0, 0 );
}
