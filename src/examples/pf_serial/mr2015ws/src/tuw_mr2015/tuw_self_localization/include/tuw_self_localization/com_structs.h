#ifndef COM_STRUCTS_H
#define COM_STRUCTS_H

#include "fixedpoint.h"

struct likelihoodLookupTable {
    fixed gausspdf[256];    
};

// in order to avoid specifying communication structs twice (struct definition and loading values) in both ROS node/FPGA slave code,
// following macro hack is used for both: member(type, member name, variable containing value to be loaded when sending)
#define com_params \
    member(bool,  enable_resample, config_.enable_resample); \
    member(bool,  enable_weighting, config_.enable_weighting); \
    member(bool,  enable_update, config_.enable_update); \
    member(bool, random_beams, config_.random_beams); \
    member(fixed, alpha1, fixed(config_.alpha1) ); \
    member(fixed, alpha2, fixed(config_.alpha2) ); \
    member(fixed, alpha3, fixed(config_.alpha3) ); \
    member(fixed, alpha4, fixed(config_.alpha4) ); \
    member(fixed, alpha5, fixed(config_.alpha5) ); \
    member(fixed, alpha6, fixed(config_.alpha6) ); \
    member(fixed, z_max, fixed(config_.z_max) ); \
    member(unsigned int,   nr_of_samples, config_.nr_of_samples); \
    member(unsigned int,   nr_of_beams, config_.nr_of_beams); \
    member(fixed, used_sigma_hit, fixed(config_.sigma_hit) ); \
    member(fixed, sigma_static_position, fixed(config_.sigma_static_position) ); \
    member(fixed, sigma_static_orientation, fixed(config_.sigma_static_orientation) ); \
    member(int, resample_strategy, config_.resample_strategy); \
    member(fixed, resample_rate, fixed(config_.resample_rate) ); \
    member(likelihoodLookupTable, likelihoodLookup, currentlikelihoodLookupTable); \
    member(fixed33mat, map_tf, ftf_);
    
//     member(fixed, z_hit, fixed(config_.z_hit)); 
//     member(fixed, z_rand_over_max, fixed(config_.z_rand/config_.z_max));
//     
    
#define com_frame_data member(fixed, duration_last_update, fixed(duration_last_update_.total_microseconds() /1000000.)); \
    member(fixed, v, fixed(u.v())); \
    member(fixed, w, fixed(u.w())); \
    member(unsigned int, total_beam_count, total_beam_count ); \
    member(fixed33mat, ztf, ztf );

#define com_sample \
    member(fixed, x, fixed( s->x() )); \
    member(fixed, y, fixed( s->y() )); \
    member(fixed, theta, fixed( s->theta() )); \
    member(fixed, weight, fixed( s->weight() ));
    
#define com_measurement \
    member(fixed, length, fixed(beam.length) ); \
    member(fixed, endpoint_x, fixed(beam.end_point.x()) ); \
    member(fixed, endpoint_y, fixed(beam.end_point.y()) ); \

    
    //member(fixed, angle, fixed(beam.angle) ); 
    
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
struct msgtype_measurement {
    com_measurement;
};

// struct for samples
struct msgtype_sample {
    com_sample;
};

#undef member

#endif