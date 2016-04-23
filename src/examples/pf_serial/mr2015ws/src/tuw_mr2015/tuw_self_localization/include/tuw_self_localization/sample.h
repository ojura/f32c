#ifndef SAMPLE_H
#define SAMPLE_H
#include <memory.h>
#include <tuw_geometry/tuw_geometry.h>
#include <tuw_self_localization/fixedpoint.h>

namespace tuw {
/**
 * Class to discribe a particle used in the particle filter localization
 */
class Sample;
typedef std::shared_ptr< Sample > SamplePtr;
typedef std::shared_ptr< Sample const> SampleConstPtr;
class Sample : public Pose2D {
    double weight_;    /// weight 
    unsigned int idx_; /// index (for debugging)
public:
  
    fixed fweight_;
    fixed fx_,fy_, ftheta_;
  
    Sample();
    /**  copy constructor
     * @param s
     **/
    Sample(const Sample& s);
    /**  constructor
     * @param p
     * @param weight
     **/
    Sample ( const Pose2D &p, double weight = 0 );
    /**  set sample
     * @param p
     * @param weight
     **/
    void set ( const Pose2D &p, double weight = 0 );
    /**  set sample
     * @param s
     **/
    void set ( const Sample &s);
    /**  set sample
     * @param s
     **/
    void set ( const SamplePtr &s);
    /**  set sample
     * @param x
     * @param y
     * @param theta
     * @param weight
     **/
    void set ( double x, double y, double theta, double weight = 0);
    /** @return idx  **/
    const unsigned int &idx () const;
    /** @return idx  **/
    unsigned int &idx ();
    /** @return weight  **/
    const double &weight () const;
    /** @return weight  **/
    double &weight ();
    /** Stream extraction
     * @param os outputstream
     * @param o object
     * @return stream
     **/
    friend std::ostream &operator << ( std::ostream &os, const Sample &o ) {
        os << "[" << ( Pose2D& ) o <<  ", " << o.weight() << "]";
        return os;
    }

    static bool greater ( const SamplePtr& a, const SamplePtr& b ) {
        return a->weight() > b->weight();
    }

    static bool smaller ( const SamplePtr& a, const SamplePtr& b ) {
        return a->weight() < b->weight();
    }
};

}
#endif // SAMPLE_H
