#include "tuw_self_localization/kalman_filter.h"
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tuw_geometry/linesegment2d_detector.h>
#include <iostream>
#include <cmath>
using namespace tuw;
KalmanFilter::KalmanFilter()
    : PoseFilter ( KALMAN_FILTER )
    , figure_hspace_ ( "Hough Space" ) {
}

void KalmanFilter::init ( ) {
    pose_estimated_ = pose_init_;
    P = cv::Matx<double, 3, 3> ( config_.init_sigma_location, 0, 0,
                                 0, config_.init_sigma_location, 0,
                                 0, 0, config_.init_sigma_orientation );
    reset_ = false;


}

void KalmanFilter::detect_lines ( const MeasurementLaserConstPtr &z ) {

    LineSegment2DDetector linesegment_detector;
    linesegment_detector.config_.threshold_split_neighbor = config_.line_dection_split_neighbor;
    linesegment_detector.config_.threshold_split = config_.line_dection_split_threshold;
    linesegment_detector.config_.min_length = config_.line_dection_min_length;
    linesegment_detector.config_.min_points_per_line = config_.line_dection_min_points_per_line;
    linesegment_detector.config_.min_points_per_unit = config_.line_dection_min_points_per_unit;
    measurement_local_scanpoints_.resize ( z->size() );
    for ( size_t i = 0; i < z->size(); i++ ) {
        measurement_local_scanpoints_[i] = z->tf() * z->operator[] ( i ).end_point;
    }
    measurement_linesegments_.clear();
    linesegment_detector.start ( measurement_local_scanpoints_, measurement_linesegments_ );

    measurement_match_.resize ( measurement_linesegments_.size(), -1 );
}


void KalmanFilter::plotData ( Figure &figure_map ) {
    plotMap ( figure_map );
    if ( config_.plot_hough_space ) plotHoughSpace();
}

void KalmanFilter::plotMap ( Figure &figure_map ) {
    char text[0xFF];
    cv::Scalar color;

    /// Plot known line segments (map)
    for ( size_t i = 0; i < map_linesegments_.size(); i++ ) {
        color = Figure::orange;
        figure_map.line ( map_linesegments_[i].p0(), map_linesegments_[i].p1(), color, 1 );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  map_linesegments_[i].pc(), cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, CV_AA );
        figure_map.putText ( boost::lexical_cast<std::string> ( i ),  map_linesegments_[i].pc(), cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, CV_AA );
    }

    cv::Matx33d M = pose_estimated_.tf();
    for ( size_t i = 0; i < measurement_linesegments_.size(); i++ ) {
        color = Figure::red;
        if ( measurement_match_[i] >= 0 ) color = Figure::green_dark;
        /**
        * @ToDo EKF
        * Draw the measurment mark matches
        **/

	//std::cout << "predicted_segments size " << predicted_linesegments_.size() << " " << measurement_match_[i] << std::endl;
	// tf_ * s->tf() * z->tf() * end_point;
	LineSegment2D predline = measurement_linesegments_[i];
        Point2D p0 = M*predline.p0();
        Point2D p1 = M*predline.p1();
        Point2D pc = M*predline.pc();

        figure_map.line ( p0, p1, color );
        figure_map.putText ( boost::lexical_cast<std::string> ( measurement_match_[i] ),  pc, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, CV_AA );
        figure_map.putText ( boost::lexical_cast<std::string> ( measurement_match_[i] ),  pc, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, CV_AA );
    }

    for ( size_t i = 0; i < measurement_match_.size(); i++ ) {
        if ( measurement_match_[i] >= 0 ) {
            /**
            * @ToDo EKF
            * Draw the match relation, it is up to you how you visualize the realtion
            **/
	    
	Point2D p0 = M*measurement_linesegments_[i].p0();
        Point2D p0p = M*predicted_linesegments_[measurement_match_[i]].p0();
	// Point2D p0p = map_linesegments_[measurement_match_[i]].p0();
	
	Point2D p1 = M*measurement_linesegments_[i].p1();
        Point2D p1p = M*predicted_linesegments_[measurement_match_[i]].p1();
	// Point2D p1p = map_linesegments_[measurement_match_[i]].p1();
	
	
	figure_map.line ( p0, p0p, Figure::blue );
	figure_map.line ( p1, p1p, Figure::blue );
	
        }
    }

    sprintf ( text, "%4.3fsec", duration_last_update_.total_microseconds() /1000000. );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::white,3, CV_AA );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::black,1, CV_AA );


    cv::Matx<double,2,2> Mw2m ( figure_map.Mw2m() ( 0,0 ), figure_map.Mw2m() ( 0,1 ), figure_map.Mw2m() ( 1,0 ), figure_map.Mw2m() ( 1,1 ) ),Mw2mt;
    cv::transpose(Mw2m, Mw2mt);
    
    /**
    * @ToDo EKF
    * Compute and plot the pose coaraiance in x and y direction
    * take the pose covariance P and crate a 2x2 matrix out of the x,y components
    * transform the matrix into the plot E = Mw2m*P(0:1,0:1)*Mw2m'
    * use the opencv to compute eigenvalues and eigen vectors to compute the size and orienation of the ellipse
    **/
    cv::Matx<double, 2, 2> E(P(0,0), P(0,1), P(1,0), P(1,1));
    E = Mw2m * E * Mw2mt;
    cv::Mat_<double> eigval, eigvec;
    cv::eigen ( E, eigval, eigvec );
    
    const double scale = 50;
    
    cv::RotatedRect ellipse ( ( figure_map.Mw2m() * pose_estimated_.position() ).cv(),cv::Size ( sqrt(eigval(0)),sqrt(eigval(1)) ), atan2(eigvec(0,1),eigvec(0,0))*180 / M_PI) ;
    cv::ellipse ( figure_map.view(),ellipse, Figure::magenta, 1, CV_AA );
    ///Plot estimated pose
    figure_map.symbol ( pose_estimated_, 0.5, Figure::magenta, 1 );
}
void KalmanFilter::plotHoughSpace ( ) {

    if ( figure_hspace_.initialized() == false ) {
        figure_hspace_.setLabel ( "alpha=%4.2f","rho=%4.2f" );
        figure_hspace_.init ( config_.hough_space_pixel_alpha, config_.hough_space_pixel_rho,
                              -M_PI*1.1, +M_PI*1.1,
                              0, config_.hough_space_meter_rho,
                              M_PI,
                              1, M_PI/4 );

        if ( config_.plot_hough_space ) cv::namedWindow ( figure_hspace_.title(), 1 );
        if ( config_.plot_hough_space ) {
            cv::moveWindow ( figure_hspace_.title(), 640, 20 );
        }
    }
    figure_hspace_.clear();

    Tf2D tf = figure_hspace_.Mw2m();
    cv::Rect rectSpace ( 0,0, figure_hspace_.view().cols, figure_hspace_.view().rows );
    for ( unsigned int i = 0; i < measurement_local_scanpoints_.size(); i++ ) {
        Point2D p0 = measurement_local_scanpoints_[i];
        for ( double alpha = figure_hspace_.min_x(); alpha < figure_hspace_.max_x(); alpha +=1.0/figure_hspace_.scale_x() ) {
            /**
            * @ToDo EKF
            * draw a wave with angle = [-pi...pi], r = x*cos(angle) + y *sin(angle) for every laser point [x,y].
            **/
	    Point2D point = Point2D(alpha,(p0.x() * cos(alpha) + p0.y() * sin(alpha)));
	    
            cv::Point hspace = (tf*point).cv() ; /// point in hspace
            if ( hspace.inside ( rectSpace ) ) {
            figure_hspace_.view().at<cv::Vec3b> ( hspace ) -=  cv::Vec3b ( 50,10,10 );  // changes a pixel value in hspace
            }
        }
    }


    cv::Scalar color;
    /// Plot measurement prediction
    
    for ( size_t i = 0; i < predicted_linesegments_.size(); i++ ) {
        color = Figure::orange;
        /**
        * @ToDo EKF
        * the map prediction in the hough space as a circle or dot
        **/
        Polar2D polar = predicted_linesegments_[i].toPolar();
        figure_hspace_.circle ( polar, 3, color, 1 );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, CV_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, CV_AA );
    }

    /// Plot measurement
    cv::RotatedRect ellipse;
    ellipse.angle  = 0;
    ellipse.size.width  = config_.data_association_line_alpha  * figure_hspace_.scale_x() * 2.0;
    ellipse.size.height = config_.data_association_line_rho * figure_hspace_.scale_y() * 2.0;
    for ( size_t i = 0; i < measurement_linesegments_.size(); i++ ) {
        Polar2D  polar = measurement_linesegments_[i].toPolar();
        color = Figure::blue_dark;
        /**
        * @ToDo EKF
        * Plot the measurement prediction KalmanFilter::predicted_linesegments_ with an ellipse
        * to show the data association threshold config_.data_association_line_alpha and config_.data_association_line_rho.
               **/
        ellipse.center = (tf *  polar).cv();
        cv::ellipse ( figure_hspace_.view(), ellipse, color, 1, CV_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, Figure::white,  3, CV_AA );
        figure_hspace_.putText ( boost::lexical_cast<std::string> ( i ),  polar, cv::FONT_HERSHEY_PLAIN, 0.6, color, 1, CV_AA );
    }
    cv::imshow ( figure_hspace_.title(),figure_hspace_.view() );
}

void KalmanFilter::data_association ( ) {
    if ( config_.enable_data_association == false ) return;


    /// compute the predicted measurement
    predicted_linesegments_.resize ( map_linesegments_.size() );
    Tf2D M = pose_predicted_.tf().inv();
    for ( size_t i = 0; i < predicted_linesegments_.size(); i++ ) {
        /**
        * @ToDo EKF
        * compute the mesurment prediction
        * predicted_linesegments_[i].set ( .... )
        **/
	predicted_linesegments_[i] = LineSegment2D(M*map_linesegments_[i].p0(), M*map_linesegments_[i].p1());
    }
  
    /// Match line segments in polar coordinates which are near to the robot
    Tf2D Mp2h = figure_hspace_.Mw2m();
    for ( size_t i = 0; i < measurement_linesegments_.size(); i++ ) {
        Polar2D measurement = measurement_linesegments_[i].toPolar();
	LineSegment2D measurement_cartesian = measurement_linesegments_[i];
        float dMin = FLT_MAX;
        measurement_match_[i] = -1;
	int best;
	float minscore = dMin;
        for ( size_t j = 0; j < predicted_linesegments_.size(); j++ ) {
            Polar2D prediction = predicted_linesegments_[j].toPolar();
	    LineSegment2D prediction_cartesian = predicted_linesegments_[j];
            /**
            * @ToDo EKF
            * find the best mesurment prediction idx j and store it in measurement_match_[i]
            **/
	    
  
	    double anglediff = (angle_difference(measurement.alpha(),prediction.alpha()));
	    angle_normalize(anglediff);
	    
	    if(std::abs(anglediff) > config_.data_association_line_alpha) continue;
	    
	    float rhodiff = std::abs(measurement.rho() - prediction.rho());
	    
	    if(rhodiff > config_.data_association_line_rho) continue;
	    
	    // prevent associating parts of the maximum distance laser circle which got detected as segments,
	    // this always yields horribly wrong associations
	    if(measurement.rho() > 4.7) continue;
	   

	     // endpoint distance did not work so well, prevents association of large map boundaries
	    /*if (std::min(prediction_cartesian.p0().x()*prediction_cartesian.p0().x() + prediction_cartesian.p0().y()*prediction_cartesian.p0().y(),
	      prediction_cartesian.p1().x()*prediction_cartesian.p1().x() + prediction_cartesian.p1().y()*prediction_cartesian.p1().y()) >
	      config_.data_association_distance_to_endpoints*config_.data_association_distance_to_endpoints) continue;*/
	    
	    // additional feature for choosing the best segment: segment center distance
	    // this helps very much in crowded areas of the Hough space
	    float cartesiandiffx = measurement_cartesian.pc().x()-prediction_cartesian.pc().x();
	    float cartesiandiffy = measurement_cartesian.pc().y()-prediction_cartesian.pc().y();
	    float cartesiandiff = cartesiandiffx*cartesiandiffx + cartesiandiffy*cartesiandiffy;	    

	    
	    float score = anglediff*anglediff + rhodiff*rhodiff + cartesiandiff;
	    if(score < minscore) { minscore = score;  best = j; }
        }
        
        if(minscore < 500) measurement_match_[i] = best;
    }


}

void KalmanFilter::reinitialize ( const Pose2D &p ) {
    setPoseInit ( p );
    reset();
}


void KalmanFilter::loadMap ( int width_pixel, int height_pixel, double min_y, double max_y, double min_x, double max_x, double roation, const std::string &file ) {
    init();
    cv::FileStorage fs ( file, cv::FileStorage::READ );
    cv::Mat_<double> l;
    fs["line segments"] >> l;
    map_linesegments_.resize ( l.rows );
    for ( size_t i = 0; i < map_linesegments_.size(); i++ ) {
        map_linesegments_[i].set ( l ( i,0 ),l ( i,1 ), l ( i,2 ),l ( i,3 ) );
    }
}

Pose2D KalmanFilter::localization ( const Command &u, const MeasurementConstPtr &z ) {
    detect_lines ( ( const MeasurementLaserConstPtr& ) z );
    if ( updateTimestamp ( z->stamp() ) ) {
        if ( reset_ ) init();
        prediction ( u );
        data_association ( );
        correction ( );
    }
    return pose_estimated_;
}

void KalmanFilter::setConfig ( const void *config ) {
    config_ = * ( ( tuw_self_localization::KalmanFilterConfig* ) config );
}

void KalmanFilter::prediction ( const Command &u ) {
  
    double dt = duration_last_update_.total_microseconds() /1000000.;
    x = pose_estimated_.state_vector();
    if ( config_.enable_prediction ) {

        /**
        * @ToDo EKF
        * compute KalmanFilter::xp and KalmanFilter::Pp as predicted pose and Covariance
        **/
        /// Pose prediction
        //xp = ?
        auto &v = u.v();
	auto &w = u.w();
	
        auto &theta = x(2);
	using std::sin; using std::cos;
 

	
	if(std::abs(w) > 1e-5) {

        cv::Matx <double, 3, 3> G( 1, 0, v / w * (-cos(theta) + cos(theta + w*dt) ), 
				   0, 1, v / w * ( -sin(theta) + sin(theta + w*dt)), 
				   0, 0, 1), Gt;
       
	cv::transpose(G, Gt);
	
	cv::Matx <double, 3,2> V(  (-sin(theta) + sin(theta + w*dt))/w, v*(sin(theta) - sin(theta + w*dt))/(w*w) + v * cos(theta + w*dt)*dt/w, 
				    (cos(theta) - cos(theta + w*dt))/w, -v*(cos(theta)-cos(theta+w*dt))/(w*w) + v*sin(theta+w*dt)*dt/w,
				    0, dt);
	cv::Matx <double, 2,3> Vt;
    
	cv::transpose(V, Vt);
	
        cv::Matx <double, 2,2> M( config_.alpha_1*v*v+ config_.alpha_2*w*w, 0, 
				 0,  config_.alpha_3*v*v+config_.alpha_4*w*w);
				 
	
	xp = x + cv::Vec< double, 3 >(v/w*(-sin(theta)+sin(theta+w*dt)), v/w*(cos(theta)-cos(theta+w*dt)), w*dt);
	
	Pp = G * P * Gt + V * M * Vt; }
	else {  
	  // the limits were computed with wolfram alpha
	  cv::Matx <double, 3, 3> G( 1, 0, - dt * v  * sin(theta), 
				   0, 1, dt*v*cos(theta), 
				   0, 0, 1), Gt;
       
	cv::transpose(G, Gt);
	
	cv::Matx <double, 3,2> V(  dt * cos(theta), -1/2 * dt*dt*v*sin(theta), 
				  dt * sin(theta), 1/2 * dt * dt * v* cos(theta), 
				    0, dt);
	cv::Matx <double, 2,3> Vt;
    
	cv::transpose(V, Vt);
	
        cv::Matx <double, 2,2> M( config_.alpha_1*v*v+ config_.alpha_2*w*w, 0, \
				 0,  config_.alpha_3*v*v+config_.alpha_4*w*w);
				 
	
	xp = x + cv::Vec< double, 3 >(dt *v*cos(theta), dt * v *sin(theta),0);
	
	Pp = G * P * Gt + V * M * Vt;

	  
	}
	
    } else {
        xp = x;
        Pp = P;
    }
    pose_predicted_ = xp;
}
void KalmanFilter::correction () {

    xc = pose_predicted_.state_vector();
    Pc = Pp;

    Q = Q.eye();

    /**
    * @ToDo EKF
    * Pose correction must update the KalmanFilter::xc and KalmanFilter::Pc which reprecents the corrected pose with covaraiance
    * have a look into Siegwart 2011 section 5.6.8.5 Case study: Kalman filter localization with line feature extraction
    **/
    Q = cv::Matx<double, 2,2> ( config_.sigma_alpha, 0, 0, config_.sigma_rho );
    for ( size_t idx_measurement = 0; idx_measurement < measurement_match_.size(); idx_measurement++ ) {
        int idx_map = measurement_match_[idx_measurement];
        if ( idx_map != -1 ) {
            /**
            * @ToDo EKF
            * Pose correction must update the KalmanFilter::xc and KalmanFilter::Pc which reprecents the corrected pose with covaraiance
            * have a look into Siegwart 2004 section Case study: Kalman filter localization with line feature extraction
            **/
            /// first the prediciton and the measurment into polar space and compute the distance
             cv::Matx<double, 2, 1> v, z, zk;  /// Messurment error between predition (known data) and detetion --> Siegwart
             cv::Matx<double, 2,3> H;  cv::Matx<double, 3,2> Ht;
            Polar2D pred =  map_linesegments_[idx_map].toPolar();
            double w_r =  pred.rho();
	    double robot = xc(0) * cos(pred.alpha()) + xc(1) * sin(pred.alpha());
	    
	    if(w_r > robot) {
	      zk(0) = pred.alpha() - xc(2);
	      angle_normalize(zk(0));
	      zk(1) = w_r - robot;
	      
	      H =  cv::Matx<double, 2,3>(0, 0, -1,
					 -cos(pred.alpha()), -sin(pred.alpha()), 0);
	    }
	    else {
	      zk(0) = pred.alpha() - xc(2) + M_PI;
	      angle_normalize(zk(0));
	      zk(1) = robot - w_r;
	      
	      H =  cv::Matx<double, 2,3>(0, 0, -1,
					 cos(pred.alpha()), sin(pred.alpha()), 0);
	    }
	    
	    cv::transpose(H,Ht);
	    
	    Polar2D measurement = measurement_linesegments_[idx_measurement].toPolar();
	    z(0) = measurement.alpha();
	    z(1) = measurement.rho();    
	  
	    v = z - zk;
            
            cv::Matx<double, 2,2> Si = H * Pc * Ht + Q;

            // cv::Matx<double, 1,1> d_mahalanobis   // just for debugging reasons
            cv::Matx<double, 3,2> K = Pc * Ht * Si.inv();
            cv::Matx<double, 3,1> dx = K * v;
            xc += dx;
	    
	    cv::Matx<double,3,3> KH = K*H;   
            Pc = (KH.eye() - KH) * Pc;
        }
    }

    if ( config_.enable_correction ) {
        pose_estimated_ = xc;
        P = Pc;
    } else {
        P = Pp;
        pose_estimated_ =  pose_predicted_.state_vector();
    }
}
