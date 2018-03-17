/*
 * vehicle.cpp
 *
 *  Created on: Dec 9, 2017
 *      Author: dzx
 */
#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(vector<int> config, int lane, float s, float v, float a, string state) {

  configure(config);
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;

}

Vehicle::~Vehicle() {}


Vehicle::Vehicle(vector<int> config, float x, float y, float s, float d, float yaw, float v, float a, string state){
  configure(config);
  this->x = x;
  this->y = y;
  this->s = s;
  this->goal_s = s + target_speed;
  this->d = d;
  this->lane = floor(d / lane_width);
  this->yaw = yaw;
  this->v = v;
  this->a = a;
  this->state = state;
}

Vehicle::Vehicle(vector<int> config, float x, float y, float s, float d, float dx, float dy){
  configure(config);
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->lane = floor(d / lane_width);
  this->yaw = atan2(dy, dx);
  this->v = sqrt(dx * dx + dy * dy);
  this->a = 0;
  this->state = "CS";
//  cout << "SF: x:" << this->x << " y:" << this->y << " s:" << this->s << " d:" << this->d << " ln:" << this->lane <<
//      " v:" << this->v << endl;
}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*

    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept.

    INPUT: A predictions map. This is a map using vehicle id as keys with predicted
        vehicle trajectories as values. A trajectory is a vector of Vehicle objects. The first
        item in the trajectory represents the vehicle at the current timestep. The second item in
        the trajectory represents the vehicle one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next ego vehicle state.

    Functions that will be useful:
    1. successor_states() - Uses the current state to return a vector of possible successor states for the finite
       state machine.
    2. generate_trajectory(string state, map<int, vector<Vehicle>> predictions) - Returns a vector of Vehicle objects
       representing a vehicle trajectory, given a state and predictions. Note that trajectory vectors
       might have size 0 if no possible trajectory exists for the state.
    3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory) - Included from
       cost.cpp, computes the cost for a trajectory.
    */

  cout << "state: " << state;
    vector<string> possible_states = successor_states();
    cout << " states:";
    for(auto it = possible_states.begin(); it != possible_states.end(); it++)
      cout << " " << *it;
    int best_state = 0;
    float min_cost = 99999999.9;
    vector<Vehicle> best_trajectory;
    for(int i = 0; i != possible_states.size(); i++){
        vector<Vehicle> trajectory = generate_trajectory(possible_states[i], predictions);
        if(!trajectory.empty()){
          float cost = this->cost(trajectory);
          cout << " " << possible_states[i] << ":" << cost;
          if(cost < min_cost){
            best_state = i;
            min_cost = cost;
            best_trajectory = trajectory;
          }
        }
    }
    cout << endl;
    best_trajectory.pop_back();
    return best_trajectory;
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) {
      if(lane != 0)
        states.push_back("PLCL");
      if(lane != lanes_available - 1)
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        if (lane != 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    } else if(state.compare("LCL") == 0 || state.compare("LCR") == 0){
      if(lane != goal_lane){
        states.push_back(state);
      }
    }

    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    float max_velocity_accel_limit = this->max_acceleration + this->v;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
      cout << "Vehicle ahead: l:" << vehicle_ahead.lane << " s:" << vehicle_ahead.s << " v:" << vehicle_ahead.v << endl;
        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = min(max_velocity_accel_limit, vehicle_ahead.v); //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    cout << "Lane " << lane << " kinematics: S:" << new_position << " v:" << new_velocity << " a:" << new_accel << endl;
    return{new_position, new_velocity, new_accel};

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = position_at(1);
    vector<Vehicle> trajectory = {Vehicle(this->get_config(), this->lane, this->s, this->v, this->a, this->state),
                                  Vehicle(this->get_config(),this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(this->get_config(),lane, this->s, this->v, this->a, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->get_config(),this->lane, new_s, new_v, new_a, "KL"));
    trajectory.push_back(Vehicle(this->get_config(),this->lane, trajectory.rbegin()->position_at(1), new_v, new_a, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->get_config(), this->lane, this->s, this->v, this->a, this->state)};
    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);
    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];

    } else {
        vector<float> best_kinematics;
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->get_config(), this->lane, new_s, new_v, new_a, state));
    Vehicle end_point = Vehicle(this->get_config(), new_lane, next_lane_new_kinematics[0],
                                next_lane_new_kinematics[1], next_lane_new_kinematics[2], state);
    float advancement = end_point.position_at(1) - end_point.s;
    end_point.s += sqrt(advancement * advancement - lane_width * lane_width);
    trajectory.push_back(end_point);
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[1];
        if (next_lane_vehicle.lane == new_lane && fabs(next_lane_vehicle.s - this->s) < 40.) {
          cout << "merge s:" << next_lane_vehicle.s << " v:" << next_lane_vehicle.v << " ";
            //If lane change is not possible, return empty trajectory.
            if(next_lane_vehicle.s < this->s + 20. && next_lane_vehicle.s > this->s - 10.) return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->get_config(), this->lane, this->s, this->v, this->a, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    Vehicle end_pos = Vehicle(this->get_config(), new_lane, kinematics[0], kinematics[1], kinematics[2], state);
    trajectory.push_back(end_pos);
    trajectory.push_back(Vehicle(this->get_config(), new_lane, end_pos.position_at(1), kinematics[1], kinematics[2], state));
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
  this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
    return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    cout << "Look behind " << lane << " s:" << this->s;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    if(found_vehicle) cout << "***FOUND VEHICLE!*** s:" << rVehicle.s << " v:" << rVehicle.v << endl;
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = this->goal_s + preferred_buffer;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    cout << "Look ahead " << lane << " s:" << this->s << " t:" << min_s << endl;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
//        cout << "ln:" << temp_vehicle.lane << " s:" << temp_vehicle.s << " ";
        if (temp_vehicle.lane == lane && temp_vehicle.s >= this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
            cout << it->first << " ";
        }
    } //cout << endl;
    if(found_vehicle) cout << endl << "*********FOUND VEHICLE********" << rVehicle.s << "****" << rVehicle.v << endl;
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
  vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      float next_s = position_at(i);
      float next_v = 0;
      if (i < horizon-1) {
        next_v = position_at(i+1) - s;
      }
      predictions.push_back(Vehicle(this->get_config(), this->lane, next_s, next_v, 0));
    }
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::configure(vector<int> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    lane_width = road_data[2];
    max_jerk = road_data[3];
    max_acceleration = road_data[4];
}

vector<int> Vehicle::get_config() {
  return {(int)target_speed, lanes_available, lane_width, max_jerk, max_acceleration};
}

int Vehicle::cost(vector<Vehicle> trajectory) {
  float predicted_s = trajectory.rbegin()->s;
  return this->s + 2 * target_speed - predicted_s;
}
