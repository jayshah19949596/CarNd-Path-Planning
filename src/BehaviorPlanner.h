//
// Created by jaysh on 7/8/2018.
//

#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H
#include "PredictionComponent.h"


class BehaviorPlanner {
public:

  //Converter to convert velocity from mph to m/s
  const double velocity_mph_to_ms_conv = 1609.344 / 3600;

  //Speed limit constraints
  //Speed limit
  const double safe_speed_limit = 48 * velocity_mph_to_ms_conv;

  //Minimum speed to ensure path smoother spline library gets coordinates in ascending order
  const double minimum_speed_limit = 3 * velocity_mph_to_ms_conv;

  BehaviorPlanner();

  virtual ~BehaviorPlanner();

  void DefiningBehavior(PredictionComponent pc, double &intended_velocity, float &lane_id);
};


#endif //PATH_PLANNING_BEHAVIORPLANNER_H
