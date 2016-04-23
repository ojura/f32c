#ifndef FILTER_KALMAN_H
#define FILTER_KALMAN_H

#include <memory.h>
#include <tuw_geometry/tuw_geometry.h>
#include <tuw_self_localization/pose_filter.h>
#include <tuw_self_localization/KalmanFilterConfig.h>

namespace tuw {
class KalmanFilter;
typedef std::shared_ptr< KalmanFilter > KalmanFilterPtr;
typedef std::shared_ptr< KalmanFilter const> KalmanFilterConstPtr;
/**
 * extended kalman filter localization for self-localization
 */
class KalmanFilter : public PoseFilter {
  
public:
    KalmanFilter();
    /**
     * used to plot debug data into a map
     * @param figure_map 
     **/
    void plotData ( Figure &figure_map );
    /**
     * reinitializes the system with a pose
     * @param p pose
     **/
    void reinitialize ( const Pose2D &p );
    /**
     * loads a given map yml  
     * @param width_pixel pixel size of the canvas
     * @param height_pixel pixel size of the canvas
     * @param min_x minimal x of the visualized space
     * @param max_x maximal x of the visualized space
     * @param min_y minimal y of the visualized space
     * @param max_y maximal y of the visualized space
     * @param rotation rotation of the visualized space
     * @param file yml file with line segments
     **/
    void loadMap ( int width_pixel, int height_pixel, double min_y, double max_y, double min_x, double max_x, double roation, const std::string &file );
    /**
     * starts the self-localization process and predicts the vehicles pose at the timestamp encoded into the measurement
     * @param u current control command
     * @param z measurment with a timestamp
     * @return estimated pose at the time of the measurement
     **/
    Pose2D localization ( const Command &u, const MeasurementConstPtr &z );
    /**
     * virtual function to set the config parameters
     * @param config of type tuw_self_localization::KalmanFilterConfig*
     **/
    void setConfig ( const void *config );
private:   
    /**
     * initializes the filter
     **/
    void init ( );
    /**
     * detect lines in a laser measurement
     * detected lines are stored in measurement_linesegments_
     * @param z laser measurement
     **/
    void detect_lines (const MeasurementLaserConstPtr &z );
    /**
     * plots debug data into a given figure
     **/
    void plotMap ( Figure &figure_map );
    /**
     * generates a debug image to plot the hough space of detected lines
     **/
    void plotHoughSpace ();
    /**
     * predicted step
     * @param u command
     **/
    void prediction ( const Command &u );
    /**
     * correction step
     **/
    void correction ();
    /**
     * data association to find matching line segments
     **/
    void data_association (); 
    
    Figure figure_hspace_;                                   /// figure to plot the hough space and to define the resolution
    std::vector<LineSegment2D> map_linesegments_;            /// known line segments in world coordinates 
    std::vector<LineSegment2D> predicted_linesegments_;      /// known line segments in sensor coordinates
    std::vector<LineSegment2D> measurement_linesegments_;    /// detected line segments in sensor coordinates
    std::vector<int> measurement_match_;                     /// index vector of successful matched measurements with predicted measurements
    Pose2D pose_predicted_;     /// predicted pose derived form xp
    cv::Vec<double, 3>  x;      /// state x
    cv::Matx<double, 3, 3> P;   /// covariance for x
    cv::Vec<double, 3>  xp;     /// prediction x
    cv::Matx<double, 3, 3> Pp;  /// prediction of state x
    cv::Vec<double, 3>  xc;     /// corrected x
    cv::Matx<double, 3, 3> Pc;  /// corrected covariance for x
    cv::Matx<double, 3, 3> Cp;  /// prediction of covariance for x
    cv::Matx<double, 3, 3> G;   /// Motion model derivation to x
    cv::Matx<double, 3, 3> R;   /// V*M*V'   --> Thrun
    cv::Matx<double, 3, 2> V;   /// Motion model
    cv::Matx<double, 2, 2> M;   /// Motion covariance
    cv::Matx<double, 2, 2> Q;   /// Detection covariance
    cv::Matx<double, 2, 3> H;   /// Sensor model derivation to x
    cv::Matx<double, 3, 2> K;   /// Kalman gain
    cv::Matx<double, 2, 2> Pi;  /// Inovation
    cv::Matx<double, 2, 1> v;   /// Messurment error between predition (known data) and detetion --> Siegwart
    
    std::vector<Point2D> measurement_local_scanpoints_; /// laser beam endpoints for line detection
    tuw_self_localization::KalmanFilterConfig config_; /// parameters
};
};

#endif // FILTER_KALMAN_H
