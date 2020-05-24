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
  double car_width = 3.0; // m, 2.5 + 0.5 (margin)
  double car_length = 6.0; // m, 5.0 + 1.0*2 (margin)
  //
  double delta_uncertainty_s = 2.0; // m/sec.
  double delta_uncertainty_d = 0.2; // m/sec.
  //
  // double ref_vel_mph = 49.5; // mph <-- This is the (maximum) speed we want to go by ourself
  // double ref_vel_mph = 80.0; // 49.5; // mph
  double ref_vel_mph = 200; // 49.5; // mph
  //
  double accel_max = 5.0; // m/s^2
  double accel_min = -8.0; // m/s^2
  //
  double safe_distance_factor_max = 2.0;
  double safe_distance_factor_min = 1.1;
  double safe_distance_margin = 7.0; // m
  //---------------------//

  // Variables
  //---------------------//
  int lane = 1;
  // The set speed, m/s <-- This is the speed our car forced to "follow" (track) because of traffic
  // double set_vel = ref_vel_mph * mph2mps; // m/s
  double set_vel = 0.0; // m/s
  // Previous decisions made
  int dec_lane = lane;
  double dec_speed = set_vel;
  std::vector<double> filtered_delta_s_list;
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
               &T_sample,&lane_width,&car_width,&car_length,&delta_uncertainty_s,&delta_uncertainty_d,
               &accel_max,&accel_min,
               &safe_distance_factor_max,&safe_distance_factor_min,&safe_distance_margin,
               &ref_vel_mph,&lane,&set_vel,&dec_lane,&dec_speed,&filtered_delta_s_list]
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



          std::cout << "-------------------------------" << std::endl;


          // // test, print sensor_fusion
          // for (size_t i=0; i < sensor_fusion.size(); ++i){
          //     double vx = sensor_fusion[i][3];
          //     double vy = sensor_fusion[i][4];
          //     double check_speed = sqrt(vx*vx + vy*vy);
          //     std::cout << "#" << i << "car's s = " << (double(sensor_fusion[i][5]) - car_s)
          //     << "\tcar's v = " << check_speed << std::endl;
          // }

          //---------------------------------------------------//
          // Get the size of previous_path (remained unexecuted way points)
          size_t prev_size = previous_path_x.size();
          size_t keep_prev_size = 6;
          if ( prev_size > keep_prev_size){
              prev_size = keep_prev_size;
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
          std::cout << "car_yaw = " << car_yaw << std::endl;
          //
          double ref_x_pre = ref_x;
          double ref_y_pre = ref_y;

          // Decide if using previous end-path yaw angle to determing the direction
          bool is_faking_ref = false;
          if (prev_size < 2){
              is_faking_ref = true;
          }else{
              double _delta_x = fabs(double(previous_path_x[prev_size-1]) - double(previous_path_x[0]) );
              double _delta_y = fabs(double(previous_path_y[prev_size-1]) - double(previous_path_y[0]) );
              if ((_delta_x + _delta_y) < 0.001){ // unit: m
                  is_faking_ref = true;
              }
          }
          // Assign the ref. values
          if (prev_size < 2){
              ref_x_pre = car_x - cos(ref_yaw);
              ref_y_pre = car_y - sin(ref_yaw);
          }else{
              // Redefine reference states as previous path end point
              ref_s = end_path_s;
              ref_d = end_path_d;
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              //
              ref_x_pre = previous_path_x[prev_size-2];
              ref_y_pre = previous_path_y[prev_size-2];
              double _delta_x = fabs(ref_x - ref_x_pre);
              double _delta_y = fabs(ref_y - ref_y_pre);
              int pre_i = (prev_size-2) - 1;
              while ( (_delta_x+_delta_y) < 0.001 && pre_i >= 0){ // unit: m
                  ref_x_pre = previous_path_x[pre_i];
                  ref_y_pre = previous_path_y[pre_i];
                  _delta_x = fabs(ref_x - ref_x_pre);
                  _delta_y = fabs(ref_y - ref_y_pre);
                  pre_i -= 1;
              }
              if (pre_i < 0){
                  ref_yaw = deg2rad(car_yaw);
                  ref_x_pre = car_x - cos(ref_yaw);
                  ref_y_pre = car_y - sin(ref_yaw);
              }else{
                  ref_yaw = atan2(ref_y - ref_y_pre, ref_x - ref_x_pre);
              }
              pre_i += 1; // The true index of the ref_x_pre and ref_y_pre

              end_path_speed = distance(ref_x, ref_y, ref_x_pre, ref_y_pre)/(T_sample * double(prev_size-1 - pre_i));
          }
          //---------------------------------//






          // Behavior control
          //---------------------------------//

          // Sample and simulation for each action
          //-----------//
          // The following variables are the output of the decision-making module
          // dec_lane, dec_speed
          {
              double T_sim_horizon = 10.0; // sec.
              double dT_sim = 0.02; // sec. sample every dT_sim second
              //
              // double T_rough_sim_1 = 10.0; // sec.
              // double dT_rough_sim_1 = 0.04; // sec.
              // //
              // double T_rough_sim_2 = 20.0; // sec.
              // double dT_rough_sim_2 = 0.1; // sec.

              // Other cars (for simulation and it original states)
              std::vector<double> c_obj_s, c_obj_s_ori;
              std::vector<double> c_obj_d, c_obj_d_ori;
              std::vector<double> c_obj_speed, c_obj_speed_ori;
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
              // Keep the original state
              c_obj_s_ori = c_obj_s;
              c_obj_d_ori = c_obj_d;
              c_obj_speed_ori = c_obj_speed;
              //
              size_t N_lane = 3;
              double target_s = ref_s + 30.0; // 30.0 m ahead
              // States of each action
              std::vector<double> a_pos_s(N_lane, ref_s);
              std::vector<double> a_pos_d(N_lane, ref_d);
              std::vector<double> a_vel_magnitude(N_lane);
              std::vector<double> a_vel_angle(N_lane);
              std::vector<double> a_set_vel(N_lane);
              std::vector<bool> a_is_lane_reached(N_lane, false);
              if (filtered_delta_s_list.size() != N_lane){
                  filtered_delta_s_list.resize(N_lane, 0.0);
              }
              // Initialize the velocity of ego car toward each lane's target
              for (size_t i=0; i < N_lane; ++i){
                  double target_d_i = lane_to_d(i, lane_width);
                  double target_angle = atan2((target_d_i-a_pos_d[i]) , (target_s-a_pos_s[i]));
                  a_vel_magnitude[i] = end_path_speed;
                  a_vel_angle[i] = target_angle;
                  a_set_vel[i] = set_vel; // Previous set_vel
              }

              // Marke if the action resulted in collision
              std::vector<bool> a_is_collided(N_lane, false);
              // Loop forward time
              for (double _t = dT_sim; _t < T_sim_horizon; _t += dT_sim){
                  // Fine sim. and rough sim.
                  // if (_t >= T_rough_sim_2){
                  //     dT_sim = dT_rough_sim_2;
                  // }else if (_t >= T_rough_sim_1){
                  //     dT_sim = dT_rough_sim_1;
                  // }

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

                      // Ego car move one step
                      //-----------------------------//
                      // Check if there is close frontal car
                      //--------//
                      a_set_vel[i] = cal_proper_speed(c_obj_s,
                                                      c_obj_d,
                                                      c_obj_speed,
                                                      a_pos_s[i], a_pos_d[i], (a_vel_magnitude[i]*cos(a_vel_angle[i])),
                                                      (ref_vel_mph*mph2mps),
                                                      car_width, car_length, accel_min,
                                                      safe_distance_margin,
                                                      safe_distance_factor_max, safe_distance_factor_min,
                                                      false
                                                  );
                      // Update speed
                      double delta_speed = a_set_vel[i] - a_vel_magnitude[i];
                      if (delta_speed > accel_max * dT_sim)
                           delta_speed = accel_max * dT_sim;
                      else if (delta_speed < accel_min * dT_sim) // Note: accel_min < 0.0
                           delta_speed = accel_min * dT_sim;
                      // Update speed, if abs(delta_speed) is too small,
                      // the speed will become set_vel (speed <- set_vel)
                      a_vel_magnitude[i] += delta_speed;

                      // Update pose
                      a_pos_s[i] += dT_sim * ( a_vel_magnitude[i] * cos(a_vel_angle[i]) );
                      a_pos_d[i] += dT_sim * ( a_vel_magnitude[i] * sin(a_vel_angle[i]) );
                      // Check if the target lane reached
                      if (!a_is_lane_reached[i]){
                          double lane_center_d = lane_to_d(i, lane_width);
                          if ( fabs(a_pos_d[i] - ref_d) >= fabs(lane_center_d - ref_d) ){
                              a_is_lane_reached[i] = true;
                              // Go straigntly forward
                              a_vel_angle[i] = 0.0;
                          }
                      }
                      //-----------------------------//

                      // Check collision
                      //-----------------------------//
                      for (size_t k=0; k < c_obj_s.size(); ++k){
                          // double dist_s = fabs(c_obj_s[k] - a_pos_s[i]); //  - _t*delta_uncertainty_s;
                          double delta_s = get_delta_s(c_obj_s[k], a_pos_s[i]);
                          double dist_s = fabs(delta_s);
                          double dist_d = fabs(c_obj_d[k] - a_pos_d[i]); //  - _t*delta_uncertainty_d;
                          // if ( dist_s <= car_length && dist_d <= car_width){
                          //     a_is_collided[i] = true;
                          //     break; // Save calculation time
                          // }else if ( (c_obj_s[k] <= a_pos_s[i]) && (c_obj_s[k] > a_pos_s[i] - (safe_distance_margin + car_length)) && dist_d <= car_width){
                          //     // The rear car is not within the safe distance with ego car
                          //     a_is_collided[i] = true;
                          //     break; // Save calculation time
                          // }
                          if (dist_d <= car_width){
                              if ( ( delta_s <= (car_length) ) && ( -delta_s <= (safe_distance_margin + car_length) ) ){
                                  a_is_collided[i] = true;
                                  break; // Save calculation time
                              }
                          }
                          //
                      }
                      //-----------------------------//
                  }
              }

              // Find te largest traveling s
              int lane_id_max = -1;
              double ds_max = 0.0;

              for (size_t i=0; i < N_lane; ++i){
                  // double delta_s_raw = (a_pos_s[i] - ref_s);
                  double delta_s_raw = get_delta_s(a_pos_s[i], ref_s);
                  filtered_delta_s_list[i] += 0.1*(delta_s_raw - filtered_delta_s_list[i]);
                  double delta_s = delta_s_raw;
                  // double delta_s = filtered_delta_s_list[i];
                  std::cout << "action #" << i << ": filtered_delta_s = " << delta_s << ",\t(end)speed=" << a_vel_magnitude[i]*mps2mph << "mph,\tcolided=" << a_is_collided[i] << std::endl;
                  // if ( !a_is_collided[i] ){
                  //     // We have to make sure that there is no collision
                  //     if ( delta_s > ds_max || lane_id_max < 0){
                  //         lane_id_max = i;
                  //         ds_max = delta_s;
                  //     }
                  // }
                  if ( delta_s > ds_max || lane_id_max < 0){
                      lane_id_max = i;
                      ds_max = delta_s;
                  }
              }
              //

              // Make decision (choose action)
              // Choose lane (the speed will be determined later)
              if (lane_id_max < 0){
              // if (false){
                  // All choise resulted in collision, just stay in the current lane and try it's best in braking
                  dec_lane = lane;
                  // dec_lane = dec_lane;
              }else{
                  // We got a best lane to go (go further during specified time period)
                  dec_lane = lane_id_max;
              }

              // Check if there is close frontal car and decide speed
              //--------//
              dec_speed = cal_proper_speed(c_obj_s_ori,
                                          c_obj_d_ori,
                                          c_obj_speed_ori,
                                          ref_s, ref_d, end_path_speed,
                                          (ref_vel_mph*mph2mps),
                                          car_width, car_length, accel_min,
                                          safe_distance_margin,
                                          safe_distance_factor_max, safe_distance_factor_min,
                                          true
                                          );
              //--------//

          }

          // Update targets
          // Update lane only if the car is already close enough


          // Method 0
          // if ( (dec_lane-lane) > 0 ){
          //     lane += 1; // Change one lane a time, no matter how far the lane we actually want
          // }else if ( (dec_lane-lane) < 0 ){
          //     lane -= 1;
          // }
          // // else keep as the previous one

          // Method 1
          // double target_d = lane_to_d(lane, lane_width);
          // if (fabs(target_d - ref_d) < 0.2*lane_width){
          //     // Consider close enough to the original target, can change lane again
          //     if ( (dec_lane-lane) > 0 ){
          //         lane += 1; // Change one lane a time, no matter how far the lane we actually want
          //     }else if ( (dec_lane-lane) < 0 ){
          //         lane -= 1;
          //     }
          //     // else keep as the previous one
          // }

          // Method 2
          int _current_lane = d_to_lane(ref_d, lane_width);
          if ( (dec_lane-_current_lane) > 0 ){
              lane = _current_lane + 1; // Change one lane a time, no matter how far the lane we actually want
          }else if ( (dec_lane-_current_lane) < 0 ){
              lane = _current_lane - 1;
          }else{
              lane = dec_lane; // The car is just at the dec_lane, set the target to dec_lane
          }

          // Method 3
          // int proposed_lane = lane;
          // if ( (dec_lane-proposed_lane) > 0 ){
          //     proposed_lane += 1; // Change one lane a time, no matter how far the lane we actually want
          // }else if ( (dec_lane-proposed_lane) < 0 ){
          //     proposed_lane -= 1;
          // }
          // // else keep as the previous one
          // double proposed_d = lane_to_d(proposed_lane, lane_width);
          // if (fabs(proposed_d - ref_d) <= 1.2*lane_width){
          //     lane = proposed_lane;
          // }
          // // else keep as the previous one

          set_vel = dec_speed;
          //-----------//

          std::cout << "dec_lane = " << dec_lane << std::endl;
          std::cout << "lane = " << lane << ", set_vel = " << set_vel*mps2mph << " mph" << std::endl;

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
