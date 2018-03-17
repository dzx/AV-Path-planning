/*
 * vehicle.h
 *
 *  Created on: Dec 9, 2017
 *      Author: dzx
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 12; // impacts "keep lane" behavior.

  int lane;

  float s;

  float v;

  float a;

  float target_speed;

  int lanes_available;

  int max_acceleration;

  float x;

  float y;

  float d;

  int goal_lane;

  int goal_s;

  string state;

  int max_jerk;

  int lane_width;

  float yaw;



  /**
  * Constructor
  */
  Vehicle();
  Vehicle(vector<int> config, int lane, float s, float v, float a, string state="CS");

  Vehicle(vector<int> config, float x, float y, float s, float d, float yaw, float v, float a, string state="CS");

  Vehicle(vector<int> config, float x, float y, float s, float d, float dx, float dy);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);

  void configure(vector<int> road_data);

  vector<int> get_config();

  int cost(vector<Vehicle> trajectory);

};

#endif /* VEHICLE_H_ */
