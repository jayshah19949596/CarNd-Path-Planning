//
// Created by jaysh on 7/8/2018.
//

#include "PredictionComponent.h"
#include <math.h>

PredictionComponent::PredictionComponent()
{}

PredictionComponent::~PredictionComponent() {}


void PredictionComponent::Init(vector<vector<double>> sensor_fusion)
{
  is_car_ahead = false;
  is_car_left = false;
  is_car_right = false;
  sensorFusion = sensor_fusion;
}

void PredictionComponent::DetectObjects(double car_s, float lane_id, int previous_size)
{

  for (int i = 0; i < sensorFusion.size(); i++) {
    double o_car_vx = sensorFusion[i][3];
    double o_car_vy = sensorFusion[i][4];
    double o_car_s = sensorFusion[i][5];
    double o_car_d = sensorFusion[i][6];
    float o_car_lane;

    if (o_car_d > 0 && o_car_d < lane_width)
    {
      o_car_lane = 0.0;
    }
    else if (o_car_d > lane_width && o_car_d < (lane_width * 2))
    {
      o_car_lane = 1.0;
    }
    else if (o_car_d > (lane_width * 2) && o_car_d < (lane_width * 3))
    {
      o_car_lane = 2.0;
    }
    else
    {
      o_car_lane = -1.0;
    }
    //Not interested if cars are not on the same side of road/divider
    if (o_car_lane == -1)
    {
      continue;
    }
    CheckCloseness(previous_size, car_s, lane_id, o_car_vx, o_car_vy, o_car_s, o_car_lane);
  }
}

void PredictionComponent::CheckCloseness(int previous_size, double car_s, float lane_id, double o_car_vx,
                                         double o_car_vy, double o_car_s, double o_car_lane)
{
  //Calculate the velocity and predicted Frenet s coordinate of car
  double o_car_vel = sqrt(pow(o_car_vx, 2) + pow(o_car_vy, 2));
  double o_car_s_ahead = o_car_s + (o_car_vel * simulator_reach_time * previous_size);

  //If other car is in the same lane
  if (o_car_lane == lane_id)
  {
    //If car is getting closer than the safe range
    if ((o_car_s_ahead > car_s) && ((o_car_s_ahead - car_s) < safe_range_ahead))
    {
      is_car_ahead = true;
    }
  } //If other car is in the lane right of our car
  else if ((o_car_lane - lane_id) == 1)
  {
    //If car is getting closer than the safe range either from behind or is ahead
    if (((o_car_s_ahead > car_s) && ((o_car_s_ahead - car_s) < safe_range_ahead)) ||
        ((car_s > o_car_s_ahead) && ((car_s - o_car_s_ahead) < safe_range_behind)))
    {
      is_car_right = true;
    }
  } //If other car is the lane left of our car
  else if ((o_car_lane - lane_id) == -1)
  {
    //If car is getting closer than the safe range either from behind or is ahead
    if (((o_car_s_ahead > car_s) && ((o_car_s_ahead - car_s) < safe_range_ahead)) ||
        ((car_s > o_car_s_ahead) && ((car_s - o_car_s_ahead) < safe_range_behind)))
    {
      is_car_left = true;
    }
  }
}

