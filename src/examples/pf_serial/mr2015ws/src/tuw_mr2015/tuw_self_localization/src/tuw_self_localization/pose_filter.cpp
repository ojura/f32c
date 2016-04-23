#include <tuw_self_localization/pose_filter.h>

using namespace tuw;

PoseFilter::PoseFilter ( Type type ) : reset_ ( true ), type_ ( type ), timestamp_last_update_() {
};
void PoseFilter::reset ( ) {
    reset_ = true;
}
void PoseFilter::setPoseInit ( const Pose2D &p ) {
    pose_init_ = p;
}
PoseFilter::Type  PoseFilter::getType() const {
    return type_;
}
const std::string PoseFilter::getTypeName() const {
    switch ( type_ ) {
    case PARTICLE_FILTER:
        return "PARTICLE_FILTER";
    case KALMAN_FILTER:
        return "KALMAN_FILTER";
    }
    return "NA";
}

const boost::posix_time::ptime& PoseFilter::time_last_update() const {
    return timestamp_last_update_;
}

///@return true on sussesful update, false on first use and if t is in the past
bool PoseFilter::updateTimestamp(const boost::posix_time::ptime& t)  {
    if ( timestamp_last_update_.is_not_a_date_time() ) timestamp_last_update_ = t;
    if(timestamp_last_update_ < t) {
      duration_last_update_ = t - timestamp_last_update_;
      timestamp_last_update_ = t;
      return true;
    } else {
      return false;
    }
}