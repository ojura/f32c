#ifndef FILTER_PARTICLE_H
#define FILTER_PARTICLE_H

#include <memory>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <tuw_self_localization/sample.h>
#include <tuw_self_localization/pose_filter.h>
#include <tuw_geometry/tuw_geometry.h>
#include <tuw_self_localization/ParticleFilterConfig.h>
#include <tuw_self_localization/config.h>

namespace tuw {

class ParticleFilter;
typedef std::shared_ptr< ParticleFilter > ParticleFilterPtr;
typedef std::shared_ptr< ParticleFilter const> ParticleFilterConstPtr;

/**
 * particle filter localization for self-localization
 */
class ParticleFilter : public PoseFilter{

public:
    /**
     * Constructor
     **/
    ParticleFilter(std::string export_likelihood_mapfile = std::string(""));

    /**
     * used to plot debug data into a map
     * @param figure_map 
     **/
    void plotData (Figure &figure_map);
    /**
     * starts the self-localization process and predicts the vehicles pose at the timestamp encoded into the measurement
     * @param u current control command
     * @param z measurment with a timestamp
     * @return estimated pose at the time of the measurement
     **/
    Pose2D localization ( const Command &u, const MeasurementConstPtr &z );
    /**
     * reinitializes the system with a pose
     * sets the reset flag and a init pose and the config_.initial_distribution = NORMAL_DISTRIBUTION; 
     * @param p pose
     **/
    void reinitialize (const Pose2D &p);
    /**
     * loads a given map image 
     * @param width_pixel pixel size of the canvas
     * @param height_pixel pixel size of the canvas
     * @param min_x minimal x of the visualized space
     * @param max_x maximal x of the visualized space
     * @param min_y minimal y of the visualized space
     * @param max_y maximal y of the visualized space
     * @param rotation rotation of the visualized space
     * @param file image as map 
     **/
    void loadMap ( int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y, double roation, const std::string &file );
    
    /**
     * virtual function to set the config parameters
     * @param config of type tuw_self_localization::ParticleFilterConfig*
     **/
    void setConfig ( const void *config );
private:
    int width_pixel_,  height_pixel_; /// map size in pixel
    double min_x_, max_x_, min_y_, max_y_, roation_; /// map real size and rotation
    double scale_;         /// map scale
    cv::Matx33d tf_;       /// transformation into the map
    //cv::Mat_<fixed> ftf_; // fixed point version

    #if defined(USEFPGA) || defined(USEFIXED)
    fixed33mat ftf_;
    #endif

    double samples_weight_max_;  /// highest normalized weight of a sample
    static std::random_device rd_;  /// random number device
    static std::mt19937 generator_; /// random number generator
    static std::uniform_real_distribution<double> uniform_distribution_x_;      /// uniform distribution used for generate a random x on the map
    static std::uniform_real_distribution<double> uniform_distribution_y_;      /// uniform distribution used for generate a random y on the map
    static std::uniform_real_distribution<double> uniform_distribution_theta_;  /// uniform distribution used for generate a random angle
    static std::normal_distribution<double> normal_distribution_;               /// normal distribution for generic use  
    double sigma_likelihood_field_;                                             /// sigma value used for the likelihood field

    #ifdef USEFPGA
    double zhit_likelihood_field_;
    double zrand_likelihood_field_;
    double zmax_likelihood_field_;
    #endif

    /**
     * generates a the likelihood field
     **/
    void updateLikelihoodField ();    
    /**
     * places a particle on a random pose on the map
     * @param sample
     * @param distribution_x
     * @param distribution_y
     * @param distribution_theta
     **/
    SamplePtr& uniform (SamplePtr &sample, std::uniform_real_distribution<double> distribution_x, std::uniform_real_distribution<double> distribution_y, std::uniform_real_distribution<double> distribution_theta) const;
    /**
     * places a particle around a given pose with a given sigmas
     * @param sample
     * @param mean
     * @param sigma_position
     * @param sigma_orientation
     **/
    SamplePtr& normal  (SamplePtr &sample, const Pose2D &mean, double sigma_position, double sigma_orientation) const;   
    /**
     * initializes the filter
     **/
    void init ( );
    /**
     * places all samples normal distributed around the pose_init_
     **/
    void initNormal ();
    /**
     * places all samples uniform distributed on the map
     **/
    void initUniform();
    /**
     * places all samples on a gird on the map
     **/
    void initGrid ();
    /**
     * executes a pose update on all particles
     * @param cmd command for the update
     **/
    void update ( const Command &cmd);


    void sendFPGAparams();
    /**
     * weights all particles and sorts them according to a measurement
     * @param z measurement
     **/
    void weighting (const MeasurementLaserConstPtr &z );
    /**
     * resamples particles
     * removes particles or adds particles if the number of particles changed
     **/
    void resample ();
    std::vector< SamplePtr > samples;             /// particles
    cv::Mat_<uint8_t> map_;                       /// map image as opencv matrix
    cv::Mat_<float> distance_field_pixel_;        /// distance field in pixels
    cv::Mat_<float> distance_field_;              /// distance field in meters
    cv::Mat_<float> likelihood_field_;            /// computed likelihood field
    tuw_self_localization::ParticleFilterConfig config_;  /// parameters
    std::string export_likelihood_mapfile_;
};
};

#endif // FILTER_PARTICLE_H
