#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<double> rotate(double x, double y, double pos_x, double pos_y, double angle){
  double shift_x = x - pos_x;
  double shift_y = y - pos_y;

  double rot_x = shift_x * cos(-angle) - shift_y * sin(-angle);
  double rot_y = shift_x * sin(-angle) + shift_y * cos(-angle);
  return {rot_x, rot_y};
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  string current_state = "KL";
  float current_accel = 0;
  int goal_lane = -1;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
               &current_state,&current_accel,&goal_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
//        cout << j << endl;
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
          	car_speed *=  (1.6 / 3.6);

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
          	int prev_size = previous_path_x.size();

          	vector<double> pts_x;
          	vector<double> pts_y;

          	double pos_x = car_x;
          	double pos_y = car_y;
          	double pos_s = car_s;
          	double pos_d = car_d;
          	double angle = deg2rad(car_yaw);
          	if(prev_size < 2){
          	  double prev_x = pos_x - cos(angle);
          	  double prev_y = pos_y - sin(angle);

          	  pts_x.push_back(prev_x);
          	  pts_x.push_back(pos_x);
          	  pts_y.push_back(prev_y);
          	  pts_y.push_back(pos_y);
          	}else{
          	  pos_x = previous_path_x[prev_size - 1];
          	  pos_y = previous_path_y[prev_size - 1];
          	  pos_s = end_path_s;
          	  pos_d = end_path_d;
          	  double prev_x = previous_path_x[prev_size - 2];
          	  double prev_y = previous_path_y[prev_size - 2];
          	  angle = atan2(pos_y - prev_y, pos_x - prev_x);
          	  car_speed = distance(prev_x, prev_y, pos_x, pos_y) / .02;

             pts_x.push_back(prev_x);
             pts_x.push_back(pos_x);
             pts_y.push_back(prev_y);
             pts_y.push_back(pos_y);
          	}

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	for(int i = 0; i != previous_path_x.size(); i++){
          	  next_x_vals.push_back(previous_path_x[i]);
          	  next_y_vals.push_back(previous_path_y[i]);
          	}


          	if(prev_size < 50){ // Do some planning and trajectory generation
          	  // Sensor Fusion Data, a list of all other cars on the same side of the road.
          	  auto sensor_fusion = j[1]["sensor_fusion"];

          	  map<int, Vehicle> other_cars;
          	  int max_speed = 49 * 1.6 / 3.6;
          	  vector<int> road_config = {max_speed, // target speed
          	      3, // lanes
          	      4, // lane width
          	      50, // max jerk
          	      10 // max accel
          	  };
          	  for(int i = 0; i != sensor_fusion.size(); i++){
          	    other_cars.insert(std::pair<int, Vehicle>(sensor_fusion[i][0], Vehicle(road_config, sensor_fusion[i][1], sensor_fusion[i][2],
          	                                                                           sensor_fusion[i][5], sensor_fusion[i][6], sensor_fusion[i][3], sensor_fusion[i][4])));
          	  }
          	  Vehicle us = Vehicle(road_config, car_x, car_y, pos_s, pos_d, car_yaw, car_speed, current_accel, current_state);
          	  if(current_state.compare("LCL") == 0 || current_state.compare("LCR") == 0){
          	    us.goal_lane = goal_lane;
          	  }
          	  int lane = us.lane;

          	  map<int, vector<Vehicle>> predictions;
          	  for(map<int, Vehicle>::iterator it = other_cars.begin(); it != other_cars.end(); it++){
          	    int v_id = it->first;
          	    vector<Vehicle> pred = it->second.generate_predictions();
          	    predictions[v_id] = pred;
          	  }
          	  vector<Vehicle> trajectory = us.choose_next_state(predictions);
          	  Vehicle projection;
          	  if(trajectory.empty()){
          	    projection = us;
          	    cout << "*** PLANNING FAIL! ***" << endl;
          	  }else{
          	    projection = *(trajectory.rbegin());
          	    cout << "v:" << us.v << " a:" << us.a << " l:" << us.lane << " gs:" << us.goal_s  << " TV:" << projection.v << " Ta:" << projection.a <<
          	        " Tl:" << projection.lane << endl;
          	  }
          	  current_state = projection.state;
          	  current_accel = projection.a;
          	  if(current_state.compare("LCL") == 0 || current_state.compare("LCR") == 0){
          	    goal_lane =  projection.lane;
          	  }else{
          	    goal_lane = -1;
          	  }


          	  const int NUM_POINTS = 3;
          	  cout << "car: s:" << pos_s << " d:" << pos_d << " proj: " << projection.s << projection.state <<
          	      projection.lane << endl;
          	  float waypoint_ds = 30.;
          	  if(projection.s > pos_s){
          	    waypoint_ds = (projection.s - pos_s) / NUM_POINTS * 2;
          	  }
          	  float waypoint_dd = 0.;
          	  if(projection.lane != lane){
          	    waypoint_dd = (projection.lane * 4. + 2. - pos_d)  / NUM_POINTS ;
          	  }
          	  cout << "dd: " << waypoint_dd << "Spline points ";
          	  for(int i = 0; i != pts_x.size(); i++){
          	    vector<double> rot_xy = rotate(pts_x[i], pts_y[i], pos_x, pos_y, angle);
          	    pts_x[i] = rot_xy[0];
          	    pts_y[i] = rot_xy[1];
          	    cout << "[" << pts_x[i] << "," << pts_y[i] << "]";
          	  }
          	  float spl_multiplier = 1.;
          	  for(int i = 0; i != NUM_POINTS; i++){
          	    double new_s = pos_s + waypoint_ds * spl_multiplier;
          	    double new_d;
          	    if(projection.lane == lane){
          	      double corr_d = (2 + 4 * lane) - pos_d ;
          	      corr_d = (corr_d < 0 ? -1. : 1.) * fmin(1., fabs(corr_d));
          	      new_d = pos_d + corr_d;
          	    }else{
          	      new_d = pos_d + waypoint_dd * (spl_multiplier );
          	    }
          	    spl_multiplier++;
          	    vector<double> next_wp = getXY(new_s, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	    vector<double> rot_wp = rotate(next_wp[0], next_wp[1], pos_x, pos_y, angle);
          	    pts_x.push_back(rot_wp[0]);
          	    pts_y.push_back(rot_wp[1]);
          	    cout << "[" << new_s << ":" << rot_wp[0] << "," << new_d << ":" << rot_wp[1] << "]";
          	  }
          	  vector<double> conv_xy = getXY(pos_s, pos_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	  cout << endl;

          	  tk::spline s;

          	  s.set_points(pts_x, pts_y);


          	  double target_speed = projection.v; //50.;
          	  double current_speed = us.v;
          	  double target_x = 30;// *(pts_x.rbegin());
          	  double target_y = s(target_x);
          	  double target_dist = sqrt(target_x * target_x + target_y * target_y);
          	  double x_add_on = 0;
          	  cout << "target: " << target_speed << " current speed:" << current_speed << " accel:" << current_accel
          	      << " prev_dots:" << previous_path_x.size() << endl;
          	  int remaining_points = previous_path_x.size() < 50 ? 50 - previous_path_x.size() : 0;
          	  for(int i = 1; i <= remaining_points || ((projection.lane != lane) && (x_add_on <= *(pts_x.rbegin()+1))) ; i++){
          	    if((target_speed > current_speed  && current_accel > 0 && i % 2) || (target_speed < current_speed && current_accel < 0)){
          	      float sim_accel = current_accel * .02;
          	      sim_accel = sim_accel < 0. ? fmax(sim_accel, -.08) : fmin(sim_accel, .05);
          	      current_speed += sim_accel;
          	    }
          	    double distance_inc = current_speed * .02;
          	    double N = target_dist / distance_inc;
          	    double x_point = x_add_on + target_dist / N;
          	    double y_point = s(x_point);
          	    x_add_on = x_point;
          	    double x_ref = x_point;
          	    double y_ref = y_point;

          	    x_point = x_ref * cos(angle) - y_ref * sin(angle);
          	    y_point = x_ref * sin(angle) + y_ref * cos(angle);

          	    x_point += pos_x;
          	    y_point += pos_y;
          	    next_x_vals.push_back(x_point);
          	    next_y_vals.push_back(y_point);
          	  }
          	} // else do 'thoughts and prayers' while simulator consumes previously generated lane-change path
          	json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
//          	cout << msg << std::endl;
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
