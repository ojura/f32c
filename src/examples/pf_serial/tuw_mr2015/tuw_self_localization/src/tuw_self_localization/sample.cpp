#include "tuw_self_localization/sample.h"
using namespace tuw;

Sample::Sample() : Pose2D(), weight_ ( 0 ) {};
Sample::Sample(const Sample& s): Pose2D(s), weight_(s.weight_) {};
Sample::Sample ( const Pose2D &p, double weight) : Pose2D ( p ), weight_ ( weight ){};

void Sample::set ( const Pose2D &p, double weight) {
  Pose2D::set(p), weight_ = weight;
}
void Sample::set ( double x, double y, double theta, double weight) {
  Pose2D::set(x,y,theta), weight_ = weight;
}
void Sample::set ( const Sample &s ) {
  set(s, s.weight_);
}
void Sample::set ( const SamplePtr &s) {
  Pose2D::set(*s);
}
/** @return idx  **/
const unsigned int &Sample::idx () const {
    return idx_;
}
/** @return idx  **/
unsigned int &Sample::idx () {
    return idx_;
}
/** @return weight  **/
const double &Sample::weight () const {
    return weight_;
}
/** @return weight  **/
double &Sample::weight () {
    return weight_;
}