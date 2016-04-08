#include <tuw_self_localization/particle_filter.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <random>
#include <iostream>
#include <cstdio>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <algorithm>

//#define USEFIXED
#define USEFPGA

#include <tuw_self_localization/fixedpoint.h>

using namespace tuw;



#ifdef USEFIXED
gaussian_random gauss_;
#endif




#ifdef USEFPGA
#include <tuw_self_localization/rs232.h>
#endif

std::random_device ParticleFilter::rd_;
std::mt19937 ParticleFilter::generator_ ( rd_() );
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_x_;
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_y_;
std::uniform_real_distribution<double> ParticleFilter::uniform_distribution_theta_;
std::normal_distribution<double> ParticleFilter::normal_distribution_;

ParticleFilter::ParticleFilter() :PoseFilter ( PARTICLE_FILTER ) {
    normal_distribution_ = std::normal_distribution<double> ();
    sigma_likelihood_field_ = 1.0;
}
SamplePtr& ParticleFilter::normal ( SamplePtr &sample, const Pose2D &mean, double sigma_position, double sigma_orientation ) const {
    sample->set ( mean.x() + normal_distribution_ ( generator_ ) * sigma_position, mean.y() + normal_distribution_ ( generator_ ) * sigma_position, mean.theta() + normal_distribution_ ( generator_ ) * sigma_orientation );
    sample->normalizeOrientation();
    return sample;
}

SamplePtr& ParticleFilter::uniform ( SamplePtr &sample, std::uniform_real_distribution<double> distribution_x, std::uniform_real_distribution<double> distribution_y, std::uniform_real_distribution<double> distribution_theta ) const {
    sample->set ( distribution_x ( generator_ ),  distribution_y ( generator_ ),  distribution_theta ( generator_ ) );
    return sample;
}
void ParticleFilter::init ( ) {
    samples.resize ( config_.nr_of_samples );
    switch ( config_.initial_distribution ) {
        case NORMAL_DISTRIBUTION:
            initNormal ();
            break;
        case UNIFORM_DISTRIBUTION:
            initUniform();
            break;
        case GRID_DISTRIBUTION:
            initGrid();
            break;
        default:
            initUniform();
    };
    reset_ = false;
}


#ifdef USEFPGA
int cport_nr;

// serial helper functions
void sendInt(unsigned int p) {

    unsigned char *c =  (unsigned char *) &p;

    //std::cout << "Sending " <<  (int) c[0]  << " "<< (int) c[1] << " " << (int) c[2] << " " << (int) c[3] << std::endl;
    RS232_SendBuf(cport_nr,  (unsigned char *) &p, 4);
}

int receiveInt() {

    unsigned char c[4];
    int p = 0;

    while(p < 4) {
        p += RS232_PollComport(cport_nr, c+p, 4-p);
        usleep(100 * 1000);
        //cout << "waiting " << p << endl;
    }

    //std::cout <<  (int) c[0]  << " "<< (int) c[1] << " " << (int) c[2] << " " << (int) c[3] << std::endl;
    return *((int *) c);
}

int receiveToken(const char *synctoken) {

    //cout << "Beginning receiveToken()" << endl;
    unsigned char c;
    int p = 0;

    while(p != 4) {

        int tries = 10;
        while(RS232_PollComport(cport_nr, &c, 1) < 1 && tries) {
            usleep(100 * 1000);
            tries--;
        }

        if(tries==0) return false;
        //cout << "Received " <<  c << " p= " << p << endl;

        if(synctoken[p] == c) p++;
        else return false;
    }

    return true;
}

void syncFPGA() {
    int synced = 0;

    while(!synced) {
        RS232_cputs(cport_nr, "sync");
        synced=receiveToken("sack");
    }

}

int comportinited = 0;
void initComport() {
    int bdrate = 115200;
    char mode[] = {'8', 'N', '1', 0};

    printf("init comport\n");
    if (comportinited) RS232_CloseComport(cport_nr);
    cport_nr = 16;
    while (RS232_OpenComport(cport_nr, bdrate, mode)) {
        printf("tried comport %d\n", cport_nr);
        cport_nr++;

        if (cport_nr == 20) exit(1);
    }
    comportinited = 1;
    printf("comport inited on %d\n", cport_nr);
}


int FPGA_params_outofdate = 1;
void ParticleFilter::sendFPGAparams() {

    syncFPGA();

    RS232_cputs(cport_nr, "pars");


    struct {
        bool enable_resample;
        bool enable_weighting;
        bool enable_update;
        fixed alpha1;
        fixed alpha2;
        fixed alpha3;
        fixed alpha4;
        fixed alpha5;
        fixed alpha6;
        int nr_of_samples;
        int nr_of_beams;
    } pars;

    #define setparam(x)  pars.x = config_.x

    setparam(enable_resample);
    setparam(enable_weighting);
    setparam(enable_update);
    setparam(alpha1);
    setparam(alpha2);
    setparam(alpha3);
    setparam(alpha4);
    setparam(alpha5);
    setparam(alpha6);
    setparam(nr_of_samples);
    setparam(nr_of_beams);

    #undef setparam

    int i;
    for(i = 0; i < sizeof(pars); i++ )
        RS232_SendBuf(cport_nr, ((unsigned char *) &pars)+i , 1 );

    printf("Sent %d chars\n", i);

    FPGA_params_outofdate = 0;

}
unsigned char FPGAgetchar() {
    unsigned char c;
    while(RS232_PollComport(cport_nr, &c, 1) < 1)
        usleep(100 * 1000);
    return c;

}

void getFPGAmsg() {

    unsigned char c;

    int endc = 0;
    const char endtoken[] = "!end!\n";
        while(endc != strlen(endtoken)) {
            c = FPGAgetchar();

            if (c == endtoken[endc]) endc++;
            else {
                for (int i = 0; i < endc; i++)
                    std::cout << endtoken[endc];
                std::cout << c;
            }
        }

}

#endif

void convertSampleToFixed(const SamplePtr& s) {
    s->fx_ = s->x();
    s->fy_ = s->y();
    s->ftheta_ = s->theta();
    s->fweight_ = s->weight();
}

void convertSampleToDouble(const SamplePtr& s) {
    s->x() = s->fx_;
    s->y() = s->fy_;
    s->theta() = s->ftheta_;
    s->weight() = s->fweight_;
}

void ParticleFilter::initNormal () {
    for ( SamplePtr &s: samples ) {
        s = std::make_shared<Sample>();
        normal ( s, pose_init_, config_.sigma_init_position, config_.sigma_init_orientation );

        convertSampleToFixed(s);
    }
}

void ParticleFilter::initUniform () {
    for ( SamplePtr &s: samples ) {
        s = std::make_shared<Sample>();
        uniform ( s, uniform_distribution_x_, uniform_distribution_y_, uniform_distribution_theta_ );

        convertSampleToFixed(s);
    }
}

void ParticleFilter::reinitialize ( const Pose2D &p ) {
    setPoseInit ( p );
    config_.initial_distribution = NORMAL_DISTRIBUTION;
    reset_ = true;
}

void ParticleFilter::initGrid () {
    float angle_division = 16;
    samples.resize ( config_.nr_of_samples,  std::make_shared<Sample>() );
    int i = 0;
    double samples_per_angle = config_.nr_of_samples / angle_division;
    double A = ( max_x_ - min_x_ ) * ( max_y_ - min_y_ );
    double samples_per_m2 = samples_per_angle / A ;
    double d =  1.0 / sqrt ( samples_per_m2 );
    for ( double x = min_x_ + d/2.; x < max_x_; x+=d ) {
        for ( double y = min_y_ + d/2.; y < max_y_; y+=d ) {
            for ( double theta = -M_PI; theta < M_PI; theta += ( 2.*M_PI ) / angle_division ) {
                if ( i  < config_.nr_of_samples ) {
                    samples[i] = std::make_shared<Sample>();
                    samples[i]->set ( x,y,theta );
                    samples[i]->idx() = i;
                    convertSampleToFixed(samples[i]);
                    i++;
                } else {
                    std::cout << config_.nr_of_samples<< " : " << i;
                }
            }
        }
    }

}

void ParticleFilter::update ( const Command &u ) {

    boost::posix_time::time_duration duration = duration_last_update_ + boost::posix_time::millisec ( config_.forward_prediction_time * 1000 );

    double dx, dy, dtheta, dt = duration.total_microseconds() /1000000.;

#ifdef USEFIXED

    fixed fdt = dt;
    for ( SamplePtr s : samples ) {
        /**
        * @ToDo MotionModel
        * implement the forward sample_motion_velocity alogrithm and be aware that w can be zero!!
	* use the config_.alpha1 - config_.alpha6 as noise parameters
        **/

        double v = u.v(), w = u.w();
        // v i w su upravljacki signali koji dolaze izvana + gaussov random

        fixed fv_w = fixed(v/w);

        fixed fv = gauss_.generate(v, config_.alpha1*v*v + config_.alpha2*w*w);
        fixed fw = gauss_.generate(w, config_.alpha3*v*v + config_.alpha4*w*w);
        fixed fgamma = gauss_.generate(0, config_.alpha5*v*v + config_.alpha6*w*w);



        if(w!=0 && v/w < 2000) {
            s->fx_ = s->fx_ + fv_w * (  - (fsin(s->ftheta_)) + (fsin(s->ftheta_ + fw * fdt))  );
            s->fy_ = s->fy_ + fv_w * (    (fcos(s->ftheta_)) - (fcos(s->ftheta_ + fw * fdt))  );
        } else {
            s->fx_ = s->fx_  + fv * fdt * (fcos(s->ftheta_));
            s->fy_ = s->fy_ + fv * fdt * (fsin(s->ftheta_));
        }
        s->ftheta_ = s->ftheta_ + fw * fdt + fgamma;

        // interno u PF ne koristimo doubleove, ali ostatku programa exposeamo trenutne vrijednosti particlea
        // u double formatu da sve ostalo radi kao i prije...
        convertSampleToDouble(s);
    }
#else
    for ( SamplePtr s : samples ) {
        /**
        * @ToDo MotionModel
        * implement the forward sample_motion_velocity alogrithm and be aware that w can be zero!!
	* use the config_.alpha1 - config_.alpha6 as noise parameters
        **/

	double w = u.w();
	double v = u.v();
	
	v = v + normal_distribution_ ( generator_ ) * (config_.alpha1*v*v + config_.alpha2*w*w);
	
	w = w + normal_distribution_ ( generator_ ) * (config_.alpha3*v*v + config_.alpha4*w*w);
	double gamma = normal_distribution_ ( generator_ ) * (config_.alpha5*v*v + config_.alpha6*w*w);
	
	if(w!=0) {
	s->x() = s->x() + v/w * ( - sin(s->theta()) + sin(s->theta() + w * dt) ) ;
	s->y() = s->y() + v/w * ( cos(s->theta()) - cos(s->theta() + w * dt) ) ;
	} else {
	  s->x() =  s->x()  + v * dt * cos(s->theta());
	  s->y() =  s->y()  + v * dt * sin(s->theta());
	}
	s->theta() = s->theta() + w * dt + gamma;
	
    }
#endif
}

#ifdef USEFPGA
Pose2D ParticleFilter::localization ( const Command &u, const MeasurementConstPtr &z ) {
    if ( updateTimestamp ( z->stamp() ) ) {

        if(!comportinited)
            initComport();

        if(FPGA_params_outofdate)
            sendFPGAparams();

        getFPGAmsg();

        updateLikelihoodField ();
        if ( reset_ ) init();
        if ( config_.enable_resample ) resample();
        if ( config_.enable_update ) update ( u );
        if ( config_.enable_weighting ) weighting ( ( const MeasurementLaserConstPtr& ) z );
        pose_estimated_ = *samples[0];
    }
    return pose_estimated_;

}
#else
Pose2D ParticleFilter::localization ( const Command &u, const MeasurementConstPtr &z ) {
    if ( updateTimestamp ( z->stamp() ) ) {
        updateLikelihoodField ();
        if ( reset_ ) init();
        if ( config_.enable_resample ) resample();
        if ( config_.enable_update ) update ( u );
        if ( config_.enable_weighting ) weighting ( ( const MeasurementLaserConstPtr& ) z );
        pose_estimated_ = *samples[0];
    }
    return pose_estimated_;

}
#endif


MeasurementLaser *zadnji;

void ParticleFilter::plotData ( Figure &figure_map ) {

    /**
    * @ToDo SensorModel
    * plot the likelihood_field_ into figure_map.background()
    **/

    for ( int r = 0; r < likelihood_field_.rows; r++ ) {
        for ( int c = 0; c < likelihood_field_.cols; c++ ) {
            auto &ref = figure_map.background();
            cv::Vec3b &color = ref.at<cv::Vec3b>(r,c);

            uint8_t intensity = 255*likelihood_field_ (r,c);
            color[2] = 255 - intensity;
            //color[1] = intensity;


        }
        //  std::cout << std::endl;
    }

    double scale =  255.0 / samples_weight_max_ ;
    char text[0xFF];
    //std::cout << "new iter" << std::endl;
    for ( int i = samples.size()-1; i >= 0; i-- ) {
        const SamplePtr &s = samples[i];
        /**
        * @ToDo MotionModel
        * plot all samples use figure_map.symbol(....
	* 
        **/

        //std::cout << *s << std::endl;
        int p = s->weight()*scale;
        figure_map.symbol(*s, 0.15, cv::Scalar(0,0,p) );


// 	if(config_.alpha6 == 0.2)
// 	  std::cout << "okinuto" << std::endl;
// 	for ( size_t i = 0; i < zadnji->size(); i++ ) {
// 	    /**
// 	    * @ToDo MotionModel
// 	    * plot the laser data into the map and use the transformation measurement_laser_->tf() as well!!
// 	    **/ 
// 	    const MeasurementLaser::Beam &beam = zadnji->operator[] ( i );
// 	    
// 	    
// 	    Point2D end_point = beam.end_point;
// 
// 	    end_point =  s->tf() * zadnji->tf() * end_point;
// 	    
// 	    
// 	    
// 	    if(config_.alpha6 == 0.2) {
// 	      
// 	    end_point = tf_ *  end_point;
// 	    int x = end_point.x(), y = end_point.y();
// 	    
// 	    auto &ref = figure_map.background();
// 	   cv::Vec3b &color = ref.at<cv::Vec3b>(y,x);
// 	   
// 	   if((beam.length < 5-1e-3)) std::cout << beam.length << "OK uzorak "; else std::cout << beam.length << "Long uzorak ";
// 	   std::cout << likelihood_field_(y, x) << std::endl;
// 
// 	   color[0] = 255; color[1]=0; color[2] = 127;
// 	    
// 	    } else figure_map.circle ( end_point, 1, Figure::red );
// 	    
// 	}


    }
    sprintf ( text, "%4.3fsec", duration_last_update_.total_microseconds() /1000000. );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::white,3, CV_AA );
    cv::putText ( figure_map.view(), text, cv::Point ( figure_map.view().cols-100,20 ), cv::FONT_HERSHEY_PLAIN, 1, Figure::black,1, CV_AA );

    figure_map.symbol ( pose_estimated_, 0.5, Figure::magenta, 1 );
}

void ParticleFilter::setConfig ( const void *config ) {
    config_ = * ( ( tuw_self_localization::ParticleFilterConfig* ) config );


    #ifdef USEFPGA
    FPGA_params_outofdate = 1;
    std::cout << "setconfig callback" << std::endl;
    #endif

}
void ParticleFilter::loadMap ( int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y, double roation, const std::string &file ) {
    width_pixel_ = width_pixel,   height_pixel_ = height_pixel;
    min_y_ = min_y, max_y_ = max_y, min_x_ = min_x, max_x_ = max_x, roation_ = roation;
    double dx = max_x_ - min_x_;
    double dy = max_y_ - min_y_;
    double sy = height_pixel / dx;
    double sx = width_pixel  / dy;
    double oy = height_pixel / 2.0;
    double ox = width_pixel  / 2.0;
    double ca = cos( roation ), sa = sin( roation );
    if ( sx == sy ) scale_ = sy;
    else {
        std::cerr << "loadMap: nonsymetric scale!";
        return;
    }
    double owx = min_x_ + dx/2.;
    double owy = min_y_ + dy/2.;
    cv::Matx<double, 3, 3 > Tw ( 1, 0, -owx, 0, 1, -owy, 0, 0, 1 ); // translation
    cv::Matx<double, 3, 3 > Sc ( sx, 0, 0, 0, sy, 0, 0, 0, 1 ); // scaling
    cv::Matx<double, 3, 3 > Sp ( -1, 0, 0, 0, 1, 0, 0, 0, 1 );  // mirroring
    cv::Matx<double, 3, 3 > R ( ca, -sa, 0, sa, ca, 0, 0, 0, 1 ); // rotation
    cv::Matx<double, 3, 3 > Tm ( 1, 0, ox, 0, 1, oy, 0, 0, 1 ); // translation
    tf_ = Tm * R * Sp * Sc * Tw;

#ifdef USEFIXED
    toFixedArr(tf_, ftf);
#endif


    map_.create ( height_pixel_, width_pixel_ );
    distance_field_pixel_.create ( height_pixel_, width_pixel_ );
    likelihood_field_.create ( height_pixel_, width_pixel_ );
    cv::Mat image = cv::imread ( file, CV_LOAD_IMAGE_GRAYSCALE );
    cv::resize ( image, map_, cv::Size ( map_.cols, map_.rows ), cv::INTER_AREA );

    uniform_distribution_x_ =  std::uniform_real_distribution<double> ( min_x_, max_x_ );
    uniform_distribution_y_ = std::uniform_real_distribution<double> ( min_y_, max_y_ );
    uniform_distribution_theta_ = std::uniform_real_distribution<double> ( -M_PI, M_PI );

    updateLikelihoodField ();

}
void ParticleFilter::updateLikelihoodField () {

    if ( sigma_likelihood_field_ == config_.sigma_hit ) return;
    sigma_likelihood_field_ = config_.sigma_hit;
    boost::math::normal normal_likelihood_field = boost::math::normal ( 0, config_.sigma_hit );

    /**
    * @ToDo SensorModel
    * using the cv::distanceTransform and the boost::math::pdf  
    **/
    cv::distanceTransform(map_, distance_field_pixel_, CV_DIST_L2,CV_DIST_MASK_PRECISE);
    distance_field_ =  distance_field_pixel_ / scale_;


    for ( int r = 0; r < likelihood_field_.rows; r++ ) {
        for ( int c = 0; c < likelihood_field_.cols; c++ ) {
            float g =  distance_field_(r,c);
            float f = boost::math::pdf(normal_likelihood_field, g);
            likelihood_field_(r,c) = f;
        }
    }


}



void ParticleFilter::weighting ( const MeasurementLaserConstPtr &z ) {

    delete zadnji;
    zadnji = new MeasurementLaser;
    *zadnji = *z;

    if ( config_.nr_of_beams >  z->size() ) config_.nr_of_beams = z->size();
    std::vector<size_t> used_beams;
    /**
    * @ToDo SensorModel
    * the used_beams should define the index of used laser beams
    **/
    if ( config_.random_beams )  {
        /**
        * @ToDo SensorModel
        * select random beams indexes
        **/

        boost::push_back(used_beams, boost::irange(0, (int) z->size() ));
        std::random_shuffle ( used_beams.begin(), used_beams.end() );
        used_beams.resize(config_.nr_of_beams);

    } else {
        /**
        * @ToDo SensorModel
        * select equlay distributed beams indexes
        **/
        int spacing = z->size() / config_.nr_of_beams;

        int i = 0; int c = 0;
        while(c < config_.nr_of_beams) {
            used_beams.push_back(i);
            i += spacing; c++;
        }
    }


#ifdef USEFIXED

    for ( size_t idx = 0; idx < samples.size(); idx++ ) {
        SamplePtr &s = samples[idx];
        /**
        * @ToDo SensorModel
        * compute the weight for each particle
        **/
        s->fweight_ = 1.;

        for(size_t k : used_beams) {
            const MeasurementLaser::Beam &beam = z->operator[] ( k );

            if(beam.length < config_.z_max - 1e-4) {
                //std::cout << "Beam " << k << std::endl;
                //std::cout << "x, y,  theta double: " << s->x() << " " << s->y() << " " << s->theta() << std::endl;
                //std::cout << "x, y,  theta fixed: " << s->fx_ << " " << s->fy_ << " " << s->ftheta_ << std::endl;


                // beam endpoint, mozemo uzeti da dobijemo precomputeano izvana
                Point2D end_point = beam.end_point;

                /*cv::Mat_<fixed> fend_point(3,1);
                fend_point(0,0) = end_point.x();
                fend_point(1,0) = end_point.y();
                fend_point(2,0) = 1.; */

                fixed x = end_point.x(), y = end_point.y();

                //std::cout << "Testiranje kompilacije" << std::endl;

                //std::cout << "Beam endpoint "; printMat(fend_point);

                fixed stf[3][3], ztf[3][3];
                fixed c_ = fcos( s->ftheta_),  s_ = fsin( s->ftheta_ );


                // matrica transformacije za sample
                stf[0][0] = c_; stf[0][1] = -s_; stf[0][2] = s->fx_; stf[1][0] = s_; stf[1][1] = c_; stf[1][2] = s->fy_;
                stf[2][0] = 0.; stf[2][1] = 0.; stf[2][2] = 1.;

                cv::Mat_<fixed> ztf_ = toFixedMat(z->tf());

                for(int i = 0; i < 3; i++)
                    for(int j = 0; j < 3; j++)
                        ztf[i][j] = ztf_(i,j);


                //std::cout << std::endl << "STF "; printMat(stf);

                //std::cout << std::endl << "ZTF "; printMat(ztf);

                //std::cout << std::endl << "TF "; printMat(ftf_);

                //fend_point = ftf_ *  stf * ztf * fend_point;

                //std::cout << std::endl << "Endpoint "; printMat(fend_point);

                int rx = int(ftf[0][2] + ftf[0][0]*stf[0][2] + ftf[0][1]*stf[1][2] + x*(ztf[0][0]*(ftf[0][0]*stf[0][0] + ftf[0][1]*stf[1][0]) + ztf[1][0]*(ftf[0][0]*stf[0][1] + ftf[0][1]*stf[1][1])) + y*(ztf[0][1]*(ftf[0][0]*stf[0][0] + ftf[0][1]*stf[1][0]) + ztf[1][1]*(ftf[0][0]*stf[0][1] + ftf[0][1]*stf[1][1])) + ztf[0][2]*(ftf[0][0]*stf[0][0] + ftf[0][1]*stf[1][0]) + ztf[1][2]*(ftf[0][0]*stf[0][1] + ftf[0][1]*stf[1][1]));
                int ry = int(ftf[1][2] + ftf[1][0]*stf[0][2] + ftf[1][1]*stf[1][2] + x*(ztf[0][0]*(ftf[1][0]*stf[0][0] + ftf[1][1]*stf[1][0]) + ztf[1][0]*(ftf[1][0]*stf[0][1] + ftf[1][1]*stf[1][1])) + y*(ztf[0][1]*(ftf[1][0]*stf[0][0] + ftf[1][1]*stf[1][0]) + ztf[1][1]*(ftf[1][0]*stf[0][1] + ftf[1][1]*stf[1][1])) + ztf[0][2]*(ftf[1][0]*stf[0][0] + ftf[1][1]*stf[1][0]) + ztf[1][2]*(ftf[1][0]*stf[0][1] + ftf[1][1]*stf[1][1]));

                if (rx>=0 & rx < width_pixel_ & ry >= 0  & ry < height_pixel_)

                    s->fweight_ = s->fweight_ * fixed(likelihood_field_(ry, rx) * config_.z_hit + config_.z_rand/config_.z_max);

                else { //std::cout << "izletio ";
                    s->fweight_ = 0; }
            }


            //exit(0);
        }

        //s->weight()=std::exp(1.5*s->weight())-1;
        //samples_weight_sum += s->weight();
        convertSampleToDouble(s);
        std::cout << "Weight: " << s->fweight_ << std::endl;
    }

    struct cmp_sample { bool operator() (SamplePtr a, SamplePtr b) { return a->fweight_.val > b->fweight_.val; } };

    /// sort and normalize particles weights
    std::sort ( samples.begin(),  samples.end(), cmp_sample() );
    samples_weight_max_ = 0;
    for ( size_t i = 0; i < samples.size(); i++ ) {
        SamplePtr &s = samples[i];
        //s->weight() /= samples_weight_sum;
        s->idx() = i;
        if ( samples_weight_max_ < s->weight() ) samples_weight_max_ = s->weight();
    }

#else
    double samples_weight_sum = 0;
    
    for ( size_t idx = 0; idx < samples.size(); idx++ ) {
        SamplePtr &s = samples[idx];
        /**
        * @ToDo SensorModel
        * compute the weight for each particle
        **/
        s->weight() = 1;
	
	for(size_t k : used_beams) {
	   const MeasurementLaser::Beam &beam = z->operator[] ( k );
	   
	   if(beam.length <  config_.z_max - 1e-4) {
	     //std::cout << "Beam " << k << std::endl;
	     //std::cout << "x, y,  theta double: " << s->x() << " " << s->y() << " " << s->theta() << std::endl;
	     
	      	Point2D end_point = beam.end_point;
		//std::cout << "Beam endpoint " << end_point;
		
		//std::cout << std::endl << "STF " << s->tf();
		
		//std::cout << std::endl << "ZTF " << z->tf();
		
		//std::cout << std::endl << "TF " << tf_;
		
		end_point = tf_ * s->tf() * z->tf() * end_point;
		
		//std::cout << std::endl << "Endpoint " << end_point;
		
		int x = end_point.x(), y = end_point.y();
		if (x>=0 & x < width_pixel_ & y >= 0  & y < height_pixel_)
		
		s->weight() *= likelihood_field_(y, x) * config_.z_hit + config_.z_rand/config_.z_max;
		//s->weight() *= 1;
		else {
		  s->weight() = 0; //std::cout << "izletio ";
		  
		}
	   }	
	   
	   //exit(0);
	}

	//s->weight()=std::exp(1.5*s->weight())-1;
        samples_weight_sum += s->weight();
	
	std::cout << "Weight: " << s->weight() << std::endl;
    }
    
    /// sort and normalize particles weights
    std::sort ( samples.begin(),  samples.end(), Sample::greater ); 
    samples_weight_max_ = 0;
    for ( size_t i = 0; i < samples.size(); i++ ) {
        SamplePtr &s = samples[i];
        // radi i bez toga
	    //s->weight() /= samples_weight_sum;
        s->idx() = i;
        if ( samples_weight_max_ < s->weight() ) samples_weight_max_ = s->weight();
    }

#endif

}

void ParticleFilter::resample () {
    double dt = duration_last_update_.total_microseconds() /1000000.;
    double fdt = dt;
    std::uniform_real_distribution<double> d ( 0,1 );
    std::uniform_int_distribution<size_t>  uniform_idx_des ( 0,samples.size()-1 );
    /**
    * @ToDo Resample
    * implement a resample weel
    **/

    // M is the number the samples to destroy
    int M = config_.resample_rate * samples.size(), N = samples.size();
#define MWEAKEST 0
#define LOWVARIANCE 1

    if(config_.resample_strategy == MWEAKEST) {

#ifdef USEFIXED
        auto cmp = [](SamplePtr a, SamplePtr b) { return a->fweight_.val < b->fweight_.val; };
        sort( samples.begin(), samples.end(), cmp);

        for(int i = 0, k = N - 1; i < std::min(M, N/2); i++, k--) {
            // napravi kopiju...
            samples[i] = std::make_shared<Sample> ( *samples[k] );
            // i dodaj random
            //normal ( samples[i], *samples[i], config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );

            samples[i]->fx_ = gauss_.generate(samples[k]->fx_, config_.sigma_static_position*dt);
            samples[i]->fy_ = gauss_.generate(samples[k]->fy_, config_.sigma_static_position*dt);

            samples[i]->ftheta_ = gauss_.generate(samples[k]->ftheta_, config_.sigma_static_orientation*dt);

            // update double vrijednosti
            convertSampleToDouble(samples[i]);
        }
#else
        auto cmp = [](SamplePtr a, SamplePtr b) { return a->weight() < b->weight(); };
      sort( samples.begin(), samples.end(), cmp);
      
      for(int i = 0, k = N - 1; i < std::min(M, N/2); i++, k--) {
	samples[i] = std::make_shared<Sample> ( *samples[k] );
	normal ( samples[i], *samples[i], config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
      }
#endif


    }

    else if (config_.resample_strategy == LOWVARIANCE) {

        // destroying M samples like in the first algorithm == sampling N-M samples
        // M is now the number of samples to draw!
        M = N-M;

        std::vector<SamplePtr> newsamples; newsamples.reserve(M);

        double r = d(generator_) / M;
        double c = samples[0]->weight();
        int i = 0;

        for(int m = 0; m<M; m++) {
            double U = r + m / (double) M;

            while(U>c) {
                if(i == N-1) {
                    goto escape; // can't remember when I've last used goto :)
                }
                i++;
                c += samples[i]->weight();
            }

            newsamples.push_back(std::make_shared<Sample>(*samples[i]));
            normal ( newsamples.back(), *newsamples.back(), config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
        }

        samples = newsamples;

        escape: { };


    }

    /// update number of samples
    if ( config_.nr_of_samples < samples.size() ) samples.resize ( config_.nr_of_samples );
    while ( config_.nr_of_samples > samples.size() ) {
        double p = d ( generator_ );
        size_t j = 0;
        j = rand() % samples.size();
        samples.push_back ( std::make_shared<Sample> ( *samples[j] ) );
        SamplePtr &s  = samples.back();
        normal ( s, *s, config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
    }
}
