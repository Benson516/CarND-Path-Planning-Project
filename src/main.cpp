#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "json.hpp"
//
#include "trajectory_planner.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Parameters for units transformation
//------------------------------------------//
double mps2mph = 2.23694; // m/s --> mph
double mph2mps = 0.44704; // mph --> m/s
//------------------------------------------//

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // Parameters
  //---------------------//
  double T_sample = 0.02; // 20 ms, sampling period
  double lane_width = 4.0; // m
  double car_width = 2.5; // m, 2.0 + 0.5 (margin)
  double car_length = 7.0; // m, 5.0 + 1.0*2 (margin)
  //
  double ref_vel_mph = 49.5; // mph <-- This is the (maximum) speed we want to go by ourself
  // double ref_vel_mph = 200; // 49.5; // mph
  //
  double accel_max = 5.0; // m/s^2
  double accel_min = -8.0; // m/s^2
  //---------------------//

  // Variables
  //---------------------//
  int lane = 1;
  // The set speed, m/s <-- This is the speed our car forced to "follow" (track) because of traffic
  // double set_vel = ref_vel_mph * mph2mps; // m/s
  double set_vel = 0.0; // m/s
  //---------------------//

  // Global fine maps
  std::vector<double> g_fine_maps_s;
  std::vector<double> g_fine_maps_x;
  std::vector<double> g_fine_maps_y;
  // generate_fine_map(0.5, map_waypoints_s, map_waypoints_x, map_waypoints_y,
  //                       g_fine_maps_s, g_fine_maps_x, g_fine_maps_y);


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &g_fine_maps_s, &g_fine_maps_x, &g_fine_maps_y,
               &T_sample,&lane_width,&car_width,&accel_max,&accel_min,
               &ref_vel_mph,&lane,&set_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];




          //---------------------------------------------------//
          // Get the size of previous_path (remained unexecuted way points)
          size_t prev_size = previous_path_x.size();
          if ( prev_size > 10){
              prev_size = 10;
          }
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //---------------------------------------------------//

          // Reference x, y, yaw states
          //---------------------------------//
          double ref_s = car_s;
          double ref_d = car_d;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          double end_path_speed = car_speed;
          //
          double ref_x_pre = ref_x;
          double ref_y_pre = ref_y;
          //
          if (prev_size < 2){
              ref_x_pre = car_x - cos(car_yaw);
              ref_y_pre = car_y - sin(car_yaw);
          }else{
              // Redefine reference states as previous path end point
              ref_s = end_path_s;
              ref_d = end_path_d;
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              ref_x_pre = previous_path_x[prev_size-2];
              ref_y_pre = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_pre, ref_x - ref_x_pre);
              end_path_speed = distance(ref_x, ref_y, ref_x_pre, ref_y_pre)/T_sample;
          }
          //---------------------------------//


          // Behavior control
          //---------------------------------//


          // // Classification of the cars
          // std::vector<size_t> car_left_ids;
          // std::vector<size_t> car_right_ids;
          // //
          // int car_front_id = -1;
          // double car_front_distance = -1;
          // double car_front_speed = 0.0; // m/s
          // int car_back_id = -1;
          // double car_back_distance = -1;
          // double car_back_speed = 0.0; // m/s
          //
          // for (size_t i=0; i < sensor_fusion.size(); ++i){
          //     // For each car on the road
          //     double d = sensor_fusion[i][6];
          //     // std::cout << i << ":d=" << d << std::endl;
          //     int car_lane_id = d_to_lane(d, lane_width);
          //     if (car_lane_id == lane){
          //         // The same lane
          //         double vx = sensor_fusion[i][3];
          //         double vy = sensor_fusion[i][4];
          //         double check_speed = sqrt(vx*vx + vy*vy);
          //         double check_car_s = sensor_fusion[i][5];
          //         // Prediction for the time at prev_size path point (simple):
          //         //                                 constant-speed, keep-lane
          //         check_car_s += (double(prev_size)*T_sample) * check_speed;
          //         if (check_car_s >= ref_s){
          //             // Front car
          //             // Find the closest front car (minimum distance ahead)
          //             if ((check_car_s-ref_s) < car_front_distance || car_front_id < 0){
          //                 car_front_id = i;
          //                 car_front_distance = check_car_s - ref_s;
          //                 car_front_speed = check_speed;
          //             }
          //         }else{
          //             // Rear car
          //             // Find the closest rear car (minimum distance behind)
          //             if ((ref_s-check_car_s) < car_back_distance || car_back_id < 0){
          //                 car_back_id = i;
          //                 car_back_distance = ref_s - check_car_s;
          //                 car_back_speed = check_speed;
          //             }
          //         }
          //     }else if (car_lane_id == (lane-1)){
          //         // The left lane
          //         car_left_ids.push_back(i);
          //     }else if (car_lane_id == (lane+1)){
          //         // The right lane
          //         car_right_ids.push_back(i);
          //     }
          // }
          //
          // // test, print out all the car ids found
          // //
          // std::cout << "car_left_ids = [";
          // for (size_t i=0; i < car_left_ids.size(); ++i){
          //     std::cout << car_left_ids[i] << " ";
          // }
          // std::cout << "]" << std::endl;
          // //
          // std::cout << "car_right_ids = [";
          // for (size_t i=0; i < car_right_ids.size(); ++i){
          //     std::cout << car_right_ids[i] << " ";
          // }
          // std::cout << "]" << std::endl;
          // //
          // std::cout << "Front: (id, distance, speed) = (" << car_front_id << ", " << car_front_distance << ", " << car_front_speed << ")" << std::endl;
          // std::cout << "Back:  (id, distance, speed) = (" << car_back_id << ", " << car_back_distance << ", " << car_back_speed << ")" << std::endl;
          // //

          // Sample and simulation for each action
          //-----------//
          {
              double T_sim_horizon = 3.0; // sec.
              double dT_sim = 0.02; // sec. sample every dT_sim second
              // Other cars
              std::vector<double> c_obj_s;
              std::vector<double> c_obj_d;
              std::vector<double> c_obj_speed;
              // Initialize the cars' status at (prev_size*T_sample) ahead of current time
              // Put all the sensed car into the lists
              for (size_t i=0; i < sensor_fusion.size(); ++i){
                    // We assume the car is going forward along the lane
                    // so we only consider the magnitude
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];
                    // Prediction (simple): constant-speed, keep-lane
                    check_car_s += (double(prev_size)*T_sample) * check_speed;
                    double d = sensor_fusion[i][6];
                    //
                    c_obj_s.push_back( check_car_s );
                    c_obj_d.push_back( d );
                    c_obj_speed.push_back( check_speed );
              }

              //
              size_t N_lane = 3;
              double target_s = ref_s + 30.0; // 30.0 m ahead
              // States of each action
              std::vector<double> a_pos_s(N_lane, ref_s);
              std::vector<double> a_pos_d(N_lane, ref_d);
              std::vector<double> a_vel_s(N_lane);
              std::vector<double> a_vel_d(N_lane);
              std::vector<bool> a_is_lane_reached(N_lane, false);
              // Initialize the velocity of ego car toward each lane's target
              for (size_t i=0; i < N_lane; ++i){
                  double target_d_i = lane_to_d(i, lane_width);
                  double target_angle = atan2((target_d_i-a_pos_d[i]) , (target_s-a_pos_s[i]));
                  a_vel_s[i] = end_path_speed * cos(target_angle);
                  a_vel_d[i] = end_path_speed * sin(target_angle);
              }

              std::vector<bool> a_is_collided(N_lane, false);
              // Loop forward time
              for (double _t = dT_sim; _t < T_sim_horizon; _t += dT_sim){
                  // Move other cars forward one step in time
                  for (size_t k=0; k < c_obj_s.size(); ++k){
                      c_obj_s[k] += dT_sim * c_obj_speed[k];
                  }
                  // Loop for each direction at current sim. time
                  for (size_t i=0; i < N_lane; ++i){
                      // Save the calculation if no hope for the action
                      if (a_is_collided[i]){
                          continue;
                      }

                      // Move one step
                      //-----------------------------//
                      a_pos_s[i] += dT_sim * a_vel_s[i];
                      a_pos_d[i] += dT_sim * a_vel_d[i];
                      // Check if the target lane reached
                      if (!a_is_lane_reached[i]){
                          if ( fabs(a_pos_d[i] - ref_d) >= fabs(double(i-lane))*lane_width ){
                              a_is_lane_reached[i] = true;
                              // Go straigntly forward
                              double _speed = sqrt(a_vel_s[i]*a_vel_s[i] + a_vel_d[i]*a_vel_d[i]);
                              a_vel_s[i] = _speed;
                              a_vel_d[i] = 0.0;
                          }
                      }
                      //-----------------------------//

                      // Check collision
                      //-----------------------------//
                      for (size_t k=0; k < c_obj_s.size(); ++k){
                          double dist_s = fabs(c_obj_s[k] - a_pos_s[i]);
                          double dist_d = fabs(c_obj_d[k] - d_pos_s[i]);
                          if ( dist_s <= car_length && dist_d <= car_width){
                              a_is_collided[i] = true;
                              break; // Save calculation
                          }
                      }
                      //-----------------------------//
                  }
              }
              // Find te largest traveling s

              // Make decision (choose action)

          }

          //-----------//


          // bool too_close = false;
          int front_car_id = -1;
          double front_car_distance = -1;
          double front_car_speed = 0.0; // m/s
          // Determin the following
          // - Find set_vel to use
          // - Determine if we are going to change lane
          // std::cout << "sensor_fusion.size() = " << sensor_fusion.size() << std::endl;
          for (size_t i=0; i < sensor_fusion.size(); ++i){
              // For each car on the road
              double d = sensor_fusion[i][6];
              // std::cout << i << ":d=" << d << std::endl;
              int car_lane_id = d_to_lane(d, lane_width);
              if (car_lane_id == lane){
                  // Car is at the same lane with may car
                  // TODO: need to be replaced with the collision criterion using car_width
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx + vy*vy);
                  double check_car_s = sensor_fusion[i][5];

                  // Prediction (simple): constant-speed, keep-lane
                  check_car_s += (double(prev_size)*T_sample) * check_speed;

                  double front_check_space = 30.0; // m
                  if ((check_car_s > ref_s) && ((check_car_s - ref_s) < front_check_space)){
                      // Do some logic here

                      // Find the true front car (minimum distance ahead)
                      if ((check_car_s-ref_s) < front_car_distance || front_car_id < 0){
                          front_car_id = i;
                          front_car_distance = check_car_s-ref_s;
                          front_car_speed = check_speed;
                      }

                      // set_vel = 29.5;
                      //
                      // too_close = true;
                      // lane change to the left-most one
                      if (lane > 0){
                          lane -= 1;
                      }else if( lane == 0){
                          lane += 1;
                      }
                  }
                  //
              }
              // end Checking the collision
          }

          // Manage the speed
          // if (too_close){
          //     std::cout << "too close!! speed down" << std::endl;
          //     set_vel -= 8.0 * T_sample; // -10.0 m/s^2
          // }else if ( set_vel < ref_vel_mph*mph2mps){
          //     std::cout << "All is well~ speed up" << std::endl;
          //     set_vel += 8.0 * T_sample; // 10.0 m/s^2
          // }
          // std::cout << "set_vel = " << set_vel*mps2mph << " mph" << std::endl;

          if (front_car_id >= 0){
              std::cout << "Car overhead!! slow down" << std::endl;
              set_vel = front_car_speed;
          }else{
              std::cout << "All is well~ go with ref_vel_mph" << std::endl;
              set_vel = ref_vel_mph*mph2mps;
          }
          std::cout << "set_vel = " << set_vel*mps2mph << " mph" << std::endl;

          //---------------------------------//

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

           // test
           //----------------------------//
          //  double dist_inc = 0.5;
          //  for (int i = 0; i < 50; ++i) {
          //     // double next_s = car_s + (i+1) * dist_inc;
          //     double next_s = car_s + i * dist_inc;
          //     double next_d = 6.0;
          //     vector<double> xy = getXY(next_s,next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //     next_x_vals.push_back(xy[0]);
          //     next_y_vals.push_back(xy[1]);
          // }
           //----------------------------//


           // // test, a single JMT
           // //----------------------------------------//
           // double speed_set = 25.0; // m/s
           // double T_end = 1.0; //
           // // Starts
           // std::vector<double> start_cond_s(3);
           // std::vector<double> start_cond_d(3);
           // start_cond_s[0] = car_s;
           // start_cond_s[1] = car_speed*0.44704;
           // start_cond_s[2] = 0.0;
           // start_cond_d[0] = car_d;
           // start_cond_d[1] = 0.0;
           // start_cond_d[2] = 0.0;
           //
           // // Ends
           // std::vector<double> end_cond_s(3);
           // std::vector<double> end_cond_d(3);
           // end_cond_s[0] = car_s + speed_set*T_end;
           // end_cond_s[1] = speed_set;
           // end_cond_s[2] = 0.0;
           // end_cond_d[0] = car_d;
           // end_cond_d[1] = 0.0;
           // end_cond_d[2] = 0.0;
           //
           // // JMTs
           // std::vector<double> param_s = JMT(start_cond_s, end_cond_s, T_end);
           // std::vector<double> param_d = JMT(start_cond_d, end_cond_d, T_end);
           //
           // for (size_t i=0; i < 50; ++i){
           //     double t = i*0.02;
           //     double next_s = get_JMT_value(t, param_s);
           //     double next_d = get_JMT_value(t, param_d);
           //     vector<double> xy = getXY(next_s,next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
           //     next_x_vals.push_back(xy[0]);
           //     next_y_vals.push_back(xy[1]);
           // }
           // //----------------------------------------//





        // Generate local fine-map
        //----------------------------------------//
        std::vector<double> fine_maps_s;
        std::vector<double> fine_maps_x;
        std::vector<double> fine_maps_y;
        get_local_fine_map(car_x, car_y, 0.5,
                            map_waypoints_s, map_waypoints_x, map_waypoints_y,
                            fine_maps_s, fine_maps_x, fine_maps_y);
        // After this, we can use fine map waypoints for applying to getXY()
        // NOTE: Because the Frenet-to-xy conversion will deform the space (non-isometry),
        //       it's not appropriate to do sampling before transformation, which will
        //       disturb the planed speed, especially when turning:
        //          - turning right will decrease the speed
        //          - turning left will increase the speed
        //----------------------------------------//


           // Generate spline
           //----------------------------------------//
           // Anchor points for spline
           vector<double> ptsx;
           vector<double> ptsy;
           // First two points
           ptsx.push_back(ref_x_pre);
           ptsx.push_back(ref_x);
           ptsy.push_back(ref_y_pre);
           ptsy.push_back(ref_y);


           // Add three evenly spaced points (in Frenet) ahead of starting point
           // Method 1: Use getXY() and some s-values ahead with fine_maps
           //           --> so that the car will not run off center line of lane
           // Note: target_space defines the maximum distance for lane-changing
           //-------------------//
           double target_space = 30.0; // m, note: 25 m/s * 1.0 s = 25 m < 30 m
           double p_space = 10.0; // m
           //-------------------//
           for (size_t i=0; i < 3; ++i){
               double _add_on_s =  target_space + i*p_space;
               vector<double> _next_wp = getXY( ref_s+_add_on_s, lane_to_d(lane,lane_width), fine_maps_s, fine_maps_x, fine_maps_y);
               ptsx.push_back(_next_wp[0]);
               ptsy.push_back(_next_wp[1]);
           }
           // Note: (important) ref_s+_add_on_s should not exceed the last s-value of fine_maps_s!!
           //         ^^^ If the error of m[i-1] > m[i] appeared, it's this reason.

           // Now we have totally 5 points in ptsx and ptsy
           // Change reference coordinate frame
           for (size_t i=0; i < ptsx.size(); ++i){
               double shift_x = ptsx[i] - ref_x;
               double shift_y = ptsy[i] - ref_y;
               ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
               ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
               // std::cout << "ptsx[" << i << "] = " << ptsx[i] << std::endl;
           }

           // Create a spline
           tk::spline s;
           // Insert anchor points
           s.set_points(ptsx, ptsy);


           // Push the previous_path into next vals
           for (size_t i=0; i < prev_size; ++i){
               next_x_vals.push_back(previous_path_x[i]);
               next_y_vals.push_back(previous_path_y[i]);
           }

           // Fill up the rest of the path after filling up with previous path points.
           //     - Here we always output 50 points
           // Caluculate how to sample the spline point for required velocity
           double target_x = target_space;
           double target_y = s(target_x);
           double target_dist = sqrt( target_x*target_x + target_y*target_y);
           // double N_sample = target_dist/(T_sample*set_vel);
           // double dist_inc_x = target_x/N_sample; // 0.5
           double x_add_on = 0;
           double speed_i = end_path_speed; // The initial speed is set to the last speed of the previous path
           for (size_t i=1; i <= (50-previous_path_x.size()); ++i){
               // Determin speed_i and dist_inc_x
               //-----------------------------------//
               double delta_speed = set_vel - speed_i;
               if (delta_speed > accel_max * T_sample)
                    delta_speed = accel_max * T_sample;
               else if (delta_speed < accel_min * T_sample) // Note: accel_min < 0.0
                    delta_speed = accel_min * T_sample;
               // Update speed_i, if abs(delta_speed) is too small,
               // the speed_i will become set_vel (speed_i <- set_vel)
               speed_i += delta_speed;
               // if ( speed_i < set_vel)
               //      speed_i += accel_max * T_sample; // Speed up
               // else if (speed_i > set_vel)
               //      speed_i += accel_min * T_sample; // Speed down. Note: accel_min < 0.0
               double dist_inc_x = (target_x/target_dist) * (T_sample*speed_i);
               //-----------------------------------//
               //
               double x_local = x_add_on + dist_inc_x;
               double y_local = s(x_local);
               x_add_on = x_local;
               // Coordinate transformation, from local frame to global frame
               double x_point = ref_x + ( x_local * cos(ref_yaw) - y_local * sin(ref_yaw) );
               double y_point = ref_y + ( x_local * sin(ref_yaw) + y_local * cos(ref_yaw) );
               // Add point to path
               next_x_vals.push_back( x_point );
               next_y_vals.push_back( y_point );
           }
           //----------------------------------------//

          //  // Constant speed path with fine curve
          //  //----------------------------//
          //   double dist_inc = T_sample*set_vel; // 0.5
          //  //  // std::cout << "dist_inc = " << dist_inc << std::endl;
          //  //  for (int i = 0; i < 50; ++i) {
          //  //     double next_s = car_s + (i+1) * dist_inc;
          //  //     double next_d = lane_to_d(lane, lane_width);
          //  //     std::vector<double> xy = getXY(next_s,next_d, fine_maps_s, fine_maps_x, fine_maps_y);
          //  //     next_x_vals.push_back(xy[0]);
          //  //     next_y_vals.push_back(xy[1]);
          //  // }
          //  for (int i = 0; i < (50-prev_size); ++i) {
          //     // double next_s = car_s + (i+1) * dist_inc;
          //     double next_s = ref_s + (i+1) * dist_inc;
          //     double next_d = lane_to_d(lane, lane_width);
          //     std::vector<double> xy = getXY(next_s,next_d, fine_maps_s, fine_maps_x, fine_maps_y);
          //     // std::vector<double> xy = getXY(next_s,next_d, g_fine_maps_s, g_fine_maps_x, g_fine_maps_y);
          //     next_x_vals.push_back(xy[0]);
          //     next_y_vals.push_back(xy[1]);
          // }
           //----------------------------//



          //---------------------------------------------------//
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
