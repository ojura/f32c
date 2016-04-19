#ifndef COM_STRUCTS_H
#define COM_STRUCTS_H

struct likelihoodLookuptable {
    fixed gausspdf[256];    
};

// in order to avoid specifying communication structs twice (struct definition and loading values) in both ROS node/FPGA slave code,
// following macro hack is used for both: member(type, member name, variable containing value to be loaded when sending)
#define com_params \
    member(bool,  enable_resample, config_.enable_resample); \
    member(bool,  enable_weighting, config_.enable_weighting); \
    member(bool,  enable_update, config_.enable_update); \
    member(fixed, alpha1, config_.alpha1); \
    member(fixed, alpha2, config_.alpha2); \
    member(fixed, alpha3, config_.alpha3); \
    member(fixed, alpha4, config_.alpha4); \
    member(fixed, alpha5, config_.alpha5); \
    member(fixed, alpha6, config_.alpha6); \
    member(unsigned int,   nr_of_samples, config_.nr_of_samples); \
    member(unsigned int,   nr_of_beams, config_.nr_of_beams); \
    member(fixed, used_sigma_hit, config_.sigma_hit); \
    member(fixed, sigma_static_position, config_.sigma_static_position); \
    member(fixed, sigma_static_orientation, config_.sigma_static_orientation); \
    member(int, resample_strategy, config_.resample_strategy); \
    member(fixed, resample_rate, config_.resample_rate); \
    member(likelihoodLookuptable, likelihoodLookup, currentlikelihoodLookuptable);

#define com_frame_data member(fixed, duration_last_update, fixed(duration_last_update_.total_microseconds() /1000000.)); \
    member(fixed, v, fixed(u.v())); \
    member(fixed, w, fixed(u.w())); \
    member(unsigned int, total_beam_count, total_beam_count )

#define com_sample \
    member(fixed, x, fixed( s->x() )); \
    member(fixed, y, fixed( s->y() )); \
    member(fixed, theta, fixed( s->theta() )); \
    member(fixed, weight, fixed( s->weight() ));
    
#define com_fixed_measurement \
    member(fixed, length,0); \
    member(fixed, angle,0);

#define member(t,x,y) t x;

// singleton structure for storing algorithm parameters
static struct {
    com_params;
} msg_params;

// singleton structure for storing current frame data
static struct {
    com_frame_data;
} msg_frame_data;


// struct for measurement
struct msgtype_fixed_measurement {
    com_fixed_measurement;
};


// struct for samples
struct msgtype_sample {
    com_sample;
};

#undef member
    



#endif