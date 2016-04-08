#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <tuw_geometry/tuw_geometry.h>
#include <tuw_local_planner/LocalPlannerConfig.h>

namespace tuw {
/**
 * Robot class
 */
class LocalPlanner {
public:
    enum ControlMode {
        STOP = 0,
        DEMO = 1,
        WANDERER = 2,
        WANDERER2 = 3,
        WALL_FOLLOWING = 4,
        WALL_FOLLOWING2 = 5,
        GOTO = 6,
        GOTO2 = 7
    };
    static std::map<ControlMode, std::string> ControlModeName_; 
    
    
    LocalPlanner(const std::string &ns); /// Constructor
    void init();                         /// initialization
    void ai();                           /// artificial intelligence calls a behaviour
    void plot();                         /// plots sensor input

protected:

    Command cmd_;  /// output variables  v, w
    unsigned long loop_count_; /// counts the filter cycles

    MeasurementLaser measurement_laser_;    /// laser measurements

    Figure figure_local_;  /// Figure for data visualization

    void ai_demo();        /// Demo behaviour
    void ai_wanderer();    /// Wanderer behaviour
    void ai_wanderer2();   /// Wanderer behaviour
    void plotLocal();      /// plots sensor input in robot coordinates
    tuw_local_planner::LocalPlannerConfig config_;
};
}

#endif // PLANNER_LOCAL_H

