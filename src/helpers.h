#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  // int prev_wp = -1;
  //
  // while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
  //   ++prev_wp;
  // }
  //
  // int wp2 = (prev_wp+1)%maps_x.size();

    int prev_wp = -1;
    int wp2 = -1;
    for (size_t i=1; i < maps_s.size(); ++i){
      if (s <= maps_s[i] && s > maps_s[i-1]){
          wp2 = i;
          prev_wp = i-1;
          break;
      }else if ( maps_s[i] < maps_s[i-1] && s > maps_s[i-1]){
          wp2 = i;
          prev_wp = i-1;
          break;
      }
    }

    if (prev_wp == -1){
        wp2 = 0;
        prev_wp =  maps_s.size() -1;

    }


  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// Pre-generated fine map
bool generate_fine_map( double s_spacing,
                        const std::vector<double> &maps_s,
                        const std::vector<double> &maps_x,
                        const std::vector<double> &maps_y,
                        std::vector<double> &fine_maps_s,
                        std::vector<double> &fine_maps_x,
                        std::vector<double> &fine_maps_y)
{
    // Initialization
    fine_maps_s.resize(0);
    fine_maps_x.resize(0);
    fine_maps_y.resize(0);

    // Generate spline
    //----------------------------------------//
    // Anchor points list for spline
    std::vector<double> ptss = maps_s;
    std::vector<double> ptsx = maps_x;
    std::vector<double> ptsy = maps_y;

    // Calculate the equality s-value for id:0 after going through a cycle
    size_t eid = maps_s.size()-1;
    double dist_end_to_zero = distance(maps_x[eid], maps_y[eid], maps_x[0], maps_y[0] );
    double s_0 = maps_s[eid] + dist_end_to_zero;
    // Insert the last point (id=0)
    ptss.push_back( s_0 );
    ptsx.push_back( maps_x[0] );
    ptsy.push_back( maps_y[0] );


    // Create splines
    //--------------------------------------//
    // These splines are parametric equations: x = sx(s), y = sy(s)
    tk::spline sx,sy;
    // Insert anchor points
    sx.set_points(ptss, ptsx);
    sy.set_points(ptss, ptsy);
    //
    // double s_spacing = 0.5; // m
    double current_s = ptss[0]; // m
    while (current_s < ptss[ptss.size()-1]){
        fine_maps_s.push_back( current_s);
        fine_maps_x.push_back(sx(current_s));
        fine_maps_y.push_back(sy(current_s));
        current_s += s_spacing;
        // std::cout << "fine_maps_s.size() = " << fine_maps_s.size() << std::endl;
    }
    std::cout << "fine_maps_s.size() = " << fine_maps_s.size() << std::endl;
    //--------------------------------------//

    return true;
}

// Local fine map
bool get_local_fine_map( double car_x, double car_y,
                        double s_spacing,
                        const std::vector<double> &maps_s,
                        const std::vector<double> &maps_x,
                        const std::vector<double> &maps_y,
                        std::vector<double> &fine_maps_s,
                        std::vector<double> &fine_maps_x,
                        std::vector<double> &fine_maps_y)
{
    // Initialization
    fine_maps_s.resize(0);
    fine_maps_x.resize(0);
    fine_maps_y.resize(0);

    // Generate spline
    //----------------------------------------//
    // Anchor points list for spline
    std::vector<double> ptss;
    std::vector<double> ptsx;
    std::vector<double> ptsy;

    // Add four evenly spaced points (in Frenet) ahead of starting point
    int closest_map_wp_id = ClosestWaypoint(car_x, car_y, maps_x, maps_y);
    std::cout << "closest_map_wp_id = " << closest_map_wp_id << std::endl;

    // Calculate the equality s-value for id:0 after going through a cycle
    size_t eid = maps_s.size()-1;
    double s_0 = maps_s[eid] + distance(maps_x[eid], maps_y[eid], maps_x[0], maps_y[0] );

    // Method: Directly use map points, which are exactly at the center of lane
    // The anchor points are: {pre-waypoint, next-waypoint, 2nd-next, 3rd-next}
    int N_spline_anchor = 10; //6;
    bool is_returned = false;
    for (int i=0; i < N_spline_anchor; ++i){
        int _id = (closest_map_wp_id+i-N_spline_anchor/2) % int(maps_s.size());
        if (_id < 0){
            _id += maps_s.size();
        }
        // Determined if it's going to return to the begining
        if (_id == 0 && i > 0){
            is_returned = true;
        }
        double ptss_value =  maps_s[_id];
        if (is_returned){
            ptss_value += s_0;
        }
        // Anchor points list
        ptss.push_back( ptss_value );
        ptsx.push_back( maps_x[_id] );
        ptsy.push_back( maps_y[_id] );
    }

    // Create splines
    //--------------------------------------//
    // These splines are parametric equations: x = sx(s), y = sy(s)
    tk::spline sx,sy;
    // Insert anchor points
    sx.set_points(ptss, ptsx);
    sy.set_points(ptss, ptsy);
    //
    // double s_spacing = 0.5; // m
    double current_s = ptss[0]; // m
    while (current_s < ptss[ptss.size()-1]){
        double current_s_1 = current_s;
        if (current_s_1 >= s_0){
            current_s_1 -= s_0;
        }
        fine_maps_s.push_back( current_s_1);
        fine_maps_x.push_back(sx(current_s));
        fine_maps_y.push_back(sy(current_s));
        current_s += s_spacing;
    }
    std::cout << "fine_maps_s.size() = " << fine_maps_s.size() << std::endl;
    //--------------------------------------//

    return true;
}


#endif  // HELPERS_H
