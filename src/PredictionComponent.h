//
// Created by jaysh on 7/8/2018.
//

#ifndef PATH_PLANNING_PREDICTIONCOMPONENT_H
#define PATH_PLANNING_PREDICTIONCOMPONENT_H

#include <vector>

using namespace std;

class PredictionComponent
{
public:

  //Flag for prediction of cars in current lane of car and other lanes
  bool is_car_ahead, is_car_left, is_car_right;

  const double lane_width = 4.0;

  //Safe distance between cars constraints
  //Safe distance ahead of our car
  const int safe_range_ahead = 30;

  //Safe distance behind our car. This is used in lane shift
  const int safe_range_behind = 15;

  // Time taken by simulator to travel from current to next waypoint - 20 ms
  const double simulator_reach_time = 0.02;

  vector<vector<double >> sensorFusion;

  PredictionComponent();

  virtual ~PredictionComponent();

  void Init(vector<vector<double>> sensor_fusion);

  void DetectObjects(double car_s, float lane_id, int previous_size);

  void CheckCloseness(int previous_size, double car_s, float lane_id, double o_car_vx,
                      double o_car_vy, double o_car_s, double o_car_lane);

};


#endif //PATH_PLANNING_PREDICTIONCOMPONENT_H
