//
// Created by jaysh on 7/8/2018.
//

#include "BehaviorPlanner.h"
#include "PredictionComponent.h"

BehaviorPlanner::BehaviorPlanner(){}

BehaviorPlanner::~BehaviorPlanner(){}


void BehaviorPlanner::DefiningBehavior(PredictionComponent pc, double &intended_velocity, float &lane_id)
{
  if (pc.is_car_ahead)
  {
    //If right lane shift is possible and our car is not in rightmost lane
    if ((!pc.is_car_right) && (lane_id != 2))
    {
      lane_id += 1;
    } //If left lane shift is possible and our car is not in leftmost lane
    else if ((!pc.is_car_left) && (lane_id != 0))
    {
      lane_id -= 1;
    } //No lane change is possible, decelerate by 0.5mph or 0.22 m/s
    else
    {
      intended_velocity -= 0.2 * velocity_mph_to_ms_conv;
    }
  } //No car is ahead and the road is clear in current lane, accelerate at 0.5mph or 0.22 m/s
  else
  {
    intended_velocity += 0.2 * velocity_mph_to_ms_conv;
  }

  //Cap the speed of car to safe speed limit slightly less than speed limit
  if (intended_velocity >= safe_speed_limit)
  {
    intended_velocity = safe_speed_limit;
  }
  //Minimum speed of car is ensured to avoid spline library exception
  if (intended_velocity <= minimum_speed_limit)
  {
    intended_velocity = minimum_speed_limit;
  }

}

