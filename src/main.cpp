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

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
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

  // Car's lane. Starting at middle lane.
  double ego_lane = 1;

  unsigned int time_elapsed = 0 ;

  // Reference velocity.
  double ego_ref_vel = 0.0;
  double ego_ref_s_prev = 0.0;
  double ego_ref_dot_s_prev = 0.0;
  double ego_ref_d_prev = 0.0;
  double ego_ref_dot_d_prev = 0.0;


  h.onMessage([&ego_ref_vel, &time_elapsed, &ego_lane, &ego_ref_s_prev, &ego_ref_dot_s_prev, &ego_ref_d_prev, &ego_ref_dot_d_prev, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {



    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	 //std::cout << "car s =  " << car_s << std::endl;
          	double car_d = j[1]["d"];
          	//std::cout << "car d =  " << car_d << std::endl;
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // Provided previous path point size.
            int prev_size = previous_path_x.size();

          	vector<double> ptsx;
            vector<double> ptsy;

            double ego_ref_x = car_x;
            double ego_ref_y = car_y;
            double ego_ref_yaw = deg2rad(car_yaw);
            double ego_ref_s = car_s;
            double ego_ref_dot_s = (ego_ref_s - ego_ref_s_prev)*50;
                   ego_ref_s_prev = ego_ref_s;
            double ego_ref_double_dot_s =(ego_ref_dot_s - ego_ref_dot_s_prev)*50;
                   ego_ref_dot_s_prev = ego_ref_dot_s;

            double ego_ref_d = car_d;
            double ego_ref_dot_d = (ego_ref_d - ego_ref_d_prev)*50;
                   ego_ref_d_prev = ego_ref_d;
            double ego_ref_double_dot_d =(ego_ref_dot_d - ego_ref_dot_d_prev)*50;
                   ego_ref_dot_d_prev = ego_ref_dot_d;

            //std::cout << "prev_size =  " << prev_size << std::endl;
            //std::cout << "dot_d =  " << ego_ref_dot_d << std::endl;
            //std::cout << "double_dot_d =  " << ego_ref_double_dot_d << std::endl;

            // If no path generated yet:
            if ( prev_size < 1 ) {

                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            } else {
                // Last point of the path
                ego_ref_x = previous_path_x[prev_size - 1];
                ego_ref_y = previous_path_y[prev_size - 1];

                double ego_ref_x_prev = previous_path_x[prev_size - 2];
                double ego_ref_y_prev = previous_path_y[prev_size - 2];
                // Calculate yaw from last two points
                ego_ref_yaw = atan2(ego_ref_y-ego_ref_y_prev, ego_ref_x-ego_ref_x_prev);

                ptsx.push_back(ego_ref_x_prev);
                ptsx.push_back(ego_ref_x);

                ptsy.push_back(ego_ref_y_prev);
                ptsy.push_back(ego_ref_y);
            }

// Output last path points
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            for ( int i = 0; i < prev_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }


// Fusion data analysis

            double ego_ref_acc = 0;

            bool car_ahead = false;
            bool car_left = false;
            bool car_right = false;
            double slow_car_vel = 200;
            double slow_car_s = 1000;

            int best_lane = 1;
            vector<double> lane_full_idx(3);

            for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                float d = sensor_fusion[i][6];
                int detected_car_lane = -1;
                // Detected car lane
                if ( d > 0 && d < 4 ) {
                  detected_car_lane = 0;
                } else if ( d > 4 && d < 8 ) {
                  detected_car_lane = 1;
                } else if ( d > 8 && d < 12 ) {
                  detected_car_lane = 2;
                }
                if (detected_car_lane < 0) {
                  continue;
                }
                // Detected car speed and s
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double detected_car_vel = sqrt(vx*vx + vy*vy);
                double detected_car_s = sensor_fusion[i][5];


                if ((detected_car_s - ego_ref_s) < 100   &&  (detected_car_s - ego_ref_s)> 50 && detected_car_lane > -1)  { lane_full_idx[detected_car_lane] += (0.5/(detected_car_s-ego_ref_s + detected_car_vel)) ;}
                if ((detected_car_s - ego_ref_s)  < 49    &&  (detected_car_s - ego_ref_s) > 0 && detected_car_lane > -1) { lane_full_idx[detected_car_lane] += (1.5/(detected_car_s-ego_ref_s + detected_car_vel))  ;}


                if ( detected_car_lane == ego_lane ) {


                    if (detected_car_s - ego_ref_s >= -3 && detected_car_s - ego_ref_s < 30 ) {
                        car_ahead = true;
                        if (slow_car_s > (detected_car_s-ego_ref_s)){
                           slow_car_vel = detected_car_vel;
                           slow_car_s = detected_car_s-ego_ref_s;
                           //std::cout << "detected car s =  " << slow_car_s << std::endl;
                        }


                    }

                } else if ( detected_car_lane - ego_lane == -1 ) {
                  // Car left
                    car_left |= detected_car_s - ego_ref_s >= 0 && detected_car_s - ego_ref_s < 20  ||  ego_ref_s  - detected_car_s  >= 0 && ego_ref_s - detected_car_s < 10;

                } else if ( detected_car_lane - ego_lane == 1 ) {
                  // Car right
                    car_right |= detected_car_s - ego_ref_s >= 0 && detected_car_s - ego_ref_s < 20  || ego_ref_s  - detected_car_s  >= 0 && ego_ref_s - detected_car_s < 10;
                }
            }

// Finding the best lane
            double min_full = 1000;

            for (int a=0; a<lane_full_idx.size();a++) {
                    if (a!= ego_lane ) { lane_full_idx[a]+=0.01 ;}
            }

            for (int a=0; a<lane_full_idx.size();a++) {

                    //std::cout << "lane number" << a <<" is " << lane_full_idx[a] << std::endl;
                    if (lane_full_idx[a] < min_full){ min_full=lane_full_idx[a]; best_lane=a;}
            }


            //std::cout << "best lane is " << best_lane << std::endl;


            double ego_ref_vel_ms = ego_ref_vel/2.24 ; //mph to m/s

            //std::cout << "ego_vel = " << ego_ref_vel_ms << std::endl;
            //std::cout << "slow_car_vel = " << slow_car_vel << std::endl;
            //std::cout << "car_ahead = " << car_ahead << std::endl;

            const double MAX_SPEED = 49.5;
            const double MAX_ACC = .25;


            if (time_elapsed >0 )time_elapsed--;



            //y[i] = 1 / (1 + exp(0.3*(-(x[i]-25)))); ;
            //double sigmoid = 1 / (1 + exp(0.1*(-(150-time_elapsed)+75)));

            //std::cout << "sigmoid = " << sigmoid << std::endl;

            if ( car_ahead && slow_car_vel < ego_ref_vel_ms ) { // Car ahead


              double braking_acc = pow((ego_ref_vel_ms-slow_car_vel),2)/ (2* ((slow_car_s)));
              //std::cout << "braking acc = " << braking_acc << std::endl;
              if ((ego_ref_vel_ms-slow_car_vel)> 0) { ego_ref_acc -= min(MAX_ACC, abs(braking_acc));}
              //else {ego_ref_acc=-0.15;} // brake softly to find free lane

              if ( !car_left && ego_lane > 0 && best_lane<2 && time_elapsed==0 && slow_car_s > 15) {

                ego_lane = ego_lane - 1.0;
                time_elapsed = 150; // Go left

              } else if ( !car_right && ego_lane != 2 && best_lane>0 && time_elapsed==0 && slow_car_s > 15){

                ego_lane =ego_lane + 1.0;
                time_elapsed = 150; // Go right
              }

            } else {

              if ( ego_ref_vel < MAX_SPEED ) { // if no one around push the pedal to the metal
                ego_ref_acc += MAX_ACC;
              }

              if ( ego_lane != best_lane && !car_right && !car_left && time_elapsed==0 ) { // if nobody around go to the best lane

                  int lane_diff = ego_lane - best_lane;
                  if (lane_diff <0){

                    ego_lane =ego_lane + 1.0;
                    time_elapsed = 150;}
                  if (lane_diff >0){

                    ego_lane = ego_lane - 1.0;
                    time_elapsed = 150;}
                }


            }

// Creating new path points

            // Map coordinates of next way points with respect to the lane
            vector<double> next_p0 = getXY(ego_ref_s + 50, 2  + 4*ego_lane,  map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_p1 = getXY(ego_ref_s + 60, 2  + 4*ego_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_p2 = getXY(ego_ref_s + 90, 2  + 4*ego_lane , map_waypoints_s, map_waypoints_x, map_waypoints_y);


            ptsx.push_back(next_p0[0]);
            ptsx.push_back(next_p1[0]);
            ptsx.push_back(next_p2[0]);

            ptsy.push_back(next_p0[1]);
            ptsy.push_back(next_p1[1]);
            ptsy.push_back(next_p2[1]);


           // Path points conversion in car coordinates
            for ( int i = 0; i < ptsx.size(); i++ ) {
              double displacement_x = ptsx[i] - ego_ref_x;
              double displacement_y = ptsy[i] - ego_ref_y;

              ptsx[i] = displacement_x * cos(- ego_ref_yaw) - displacement_y * sin(-ego_ref_yaw);
              ptsy[i] = displacement_x * sin(- ego_ref_yaw) + displacement_y * cos(-ego_ref_yaw);
            }

            // Create the path spline
            tk::spline path;
            path.set_points(ptsx, ptsy);


            // XY of the path in defined x distance
            double target_x = 30.0;
            double target_y = path(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_previous = 0;

            for( int i = 0; i < (40-prev_size); i++ ) {

              ego_ref_vel += ego_ref_acc;
              if ( ego_ref_vel > MAX_SPEED ) {
                ego_ref_vel = MAX_SPEED;
              }

              double N = target_dist/(0.02*(ego_ref_vel/2.24));
              double x_point = x_previous + target_x/N;
              double y_point = path(x_point);

              //std::cout << "ego_ref_vel = " << ego_ref_vel << std::endl;

              x_previous = x_point;

              double ego_x_local = x_point;
              double ego_y_local = y_point;

               // Convert back to global coordinates

              x_point = ego_x_local * cos(ego_ref_yaw) - ego_y_local * sin(ego_ref_yaw);
              y_point = ego_x_local * sin(ego_ref_yaw) + ego_y_local * cos(ego_ref_yaw);

              x_point += ego_ref_x; //adding the present ego position
              y_point += ego_ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

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
