#include <art_planner/planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>



namespace ob = ompl::base;
namespace og = ompl::geometric;



namespace art_planner {

class Converter {

  std::shared_ptr<Planner::StateSpace> space_;

public:

  Converter(std::shared_ptr<Planner::StateSpace> space);

  ob::ScopedState<> poseRosToOmpl(const geometry_msgs::PoseStamped& pose_ros) const;

  nav_msgs::Path pathOmplToRos(const og::PathGeometric& path) const;

};



}
