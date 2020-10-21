#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "math.h"

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  struct map_waypoints
  {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<double> dx;
    std::vector<double> dy;
  } map_waypoints;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line))
  {
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
    map_waypoints.x.push_back(x);
    map_waypoints.y.push_back(y);
    map_waypoints.s.push_back(s);
    map_waypoints.dx.push_back(d_x);
    map_waypoints.dy.push_back(d_y);
  }

  const struct driving_data
  {
    double safety_dist = 20.0;
    double spline_dist = 35.0;
    double lane_width = 4.0;
    double delta_time = 0.02; // s = 20ms
    double max_speed = 22;    // speed in m/s slightly lower than 50MPH
    double max_accel = 10.0;  // m/s^2
  } dd;

  struct car_data
  {
    double ctrl_speed; // speed control for car
    int lane_id;
  } cd{0.0, 1};

  h.onMessage([&map_waypoints, &dd, &cd](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(data);

      if (s != "")
      {
        auto j = nlohmann::json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry")
        {
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /*
           * TODO: define a path made up of (x,y) points that the car will visit
           *       sequentially every .02 seconds
          */
          double ref_speed = dd.max_speed;
          car_speed *= 0.44704; // convert main car's speed from mph to m/s

          bool front_car_slower = false;
          bool left_clear = true;
          bool right_clear = true;

          // loop over all other cars on the same side of the road detected from sensor fusion
          for (int i = 0; i < sensor_fusion.size(); ++i)
          {
            // check if there are other cars in main car's lane going slower than main car
            // check if the other car's Frenet d component is within the boundaries of main car's lane
            if (cd.lane_id * dd.lane_width < sensor_fusion[i][6] &&
                sensor_fusion[i][6] < (cd.lane_id + 1) * dd.lane_width)
            {
              // check if other car is in front and closer than the end of previous path plus safety distance
              if (sensor_fusion[i][5] < (end_path_s + dd.safety_dist) &&
                  sensor_fusion[i][5] > (car_s - 5.0))
              {
                // calculate others car's absolute speed value in m/s
                double front_car_speed = std::sqrt(pow(sensor_fusion[i][3], 2.0) + pow(sensor_fusion[i][4], 2.0));
                // calculate the diference in speed between the two cars
                double delta_speed = car_speed - front_car_speed;
                // if the car in front is going slower than main car, match its speed
                if (delta_speed > 0.0)
                {
                  ref_speed = front_car_speed;
                  front_car_slower = true;
                }
              }
            }

            // check for other cars in case of potential lane change to the left
            // check that the main car is not in the leftmost lane
            if (cd.lane_id > 0 &&
                (cd.lane_id - 1) * dd.lane_width < sensor_fusion[i][6] &&
                sensor_fusion[i][6] < cd.lane_id * dd.lane_width)
            {
              // look for available space in the left lane
              if (sensor_fusion[i][5] < car_s + 2 * dd.safety_dist &&
                  sensor_fusion[i][5] > car_s - dd.safety_dist / 2)
              {
                left_clear = false;
              }
            }
            // check for other cars in case of potential lane change to the right
            // check that the main car is not in the rightmost lane
            if (cd.lane_id < 2 &&
                (cd.lane_id + 1) * dd.lane_width < sensor_fusion[i][6] &&
                sensor_fusion[i][6] < (cd.lane_id + 2) * dd.lane_width)
            {
              // look for available space in the right lane
              if ((sensor_fusion[i][5] < car_s + 2 * dd.safety_dist) &&
                  (sensor_fusion[i][5] > car_s - dd.safety_dist / 2))
              {
                right_clear = false;
              }
            }
          }

          // check if the car in front is too slow
          // change lanes if a lane change is not already in progress
          if (front_car_slower == true && car_speed < 0.9 * dd.max_speed)
          {
            std::cout << "Trying to change lanes from lane " << cd.lane_id << "." << std::endl;

            // try to change lanes to the left
            if (cd.lane_id > 0 && left_clear == true)
            {
              std::cout << "Left lane is clear, changing lanes to the left." << std::endl;
              cd.lane_id -= 1;
            }
            else if (cd.lane_id < 2 && right_clear == true)
            {
              std::cout << "Right lane is clear, changing lanes to the right." << std::endl;
              cd.lane_id += 1;
            }
          }

          // calculate path
          int path_size = previous_path_x.size();

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;
          // transfer previous path's points to new path
          for (int i = 0; i < path_size; ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double angle;
          double pos_x;
          double pos_y;
          double pos_x2;
          double pos_y2;
          double distance_increment = 0.2;

          // get previous path's end points and angles
          // if previous path is empty or too short, get the car's current position and angle
          if (path_size == 0 || path_size == 1)
          {
            // get car's current position
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            // calculate second to last position using angle
            pos_x2 = pos_x - distance_increment * cos(angle);
            pos_y2 = pos_y - distance_increment * sin(angle);
          }
          else
          {
            // get last position from previous path
            pos_x = previous_path_x[path_size - 1];
            pos_y = previous_path_y[path_size - 1];
            // get second to last position from previous path
            pos_x2 = previous_path_x[path_size - 2];
            pos_y2 = previous_path_y[path_size - 2];
            angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
          }

          std::vector<double> spline_x_vals;
          std::vector<double> spline_y_vals;
          // add last two points to spline list for better transition trajectory
          spline_x_vals.push_back(pos_x2);
          spline_y_vals.push_back(pos_y2);
          spline_x_vals.push_back(pos_x);
          spline_y_vals.push_back(pos_y);

          std::vector<double> next_point;
          // calculate spline points ahead of main car, spaced by spline_dist
          next_point = getXY(car_s + dd.spline_dist,
                             dd.lane_width / 2 + dd.lane_width * cd.lane_id,
                             map_waypoints.s, map_waypoints.x, map_waypoints.y);
          spline_x_vals.push_back(next_point[0]);
          spline_y_vals.push_back(next_point[1]);
          next_point = getXY(car_s + 2 * dd.spline_dist,
                             dd.lane_width / 2 + dd.lane_width * cd.lane_id,
                             map_waypoints.s, map_waypoints.x, map_waypoints.y);
          spline_x_vals.push_back(next_point[0]);
          spline_y_vals.push_back(next_point[1]);
          next_point = getXY(car_s + 3 * dd.spline_dist,
                             dd.lane_width / 2 + dd.lane_width * cd.lane_id,
                             map_waypoints.s, map_waypoints.x, map_waypoints.y);
          spline_x_vals.push_back(next_point[0]);
          spline_y_vals.push_back(next_point[1]);

          double shift_x;
          double shift_y;
          tk::spline spline;
          // transform spline points from map coordinates to car coordinates
          // from the perspective of the end point of the previous path
          for (int i = 0; i < spline_x_vals.size(); ++i)
          {
            shift_x = spline_x_vals[i] - pos_x;
            shift_y = spline_y_vals[i] - pos_y;
            spline_x_vals[i] = shift_x * cos(0 - angle) - shift_y * sin(0 - angle);
            spline_y_vals[i] = shift_x * sin(0 - angle) + shift_y * cos(0 - angle);
          }
          // compute spline
          spline.set_points(spline_x_vals, spline_y_vals);

          // gradually approach reference speed
          if (car_speed > ref_speed && car_speed > 0.0)
          {
            cd.ctrl_speed -= dd.max_accel * dd.delta_time;
            if (cd.ctrl_speed > dd.max_speed)
              cd.ctrl_speed = dd.max_speed;
          }
          else if (car_speed < ref_speed && ref_speed <= dd.max_speed)
          {
            cd.ctrl_speed += dd.max_accel * dd.delta_time;
            if (cd.ctrl_speed > dd.max_speed)
              cd.ctrl_speed = dd.max_speed;
          }

          // calculate x distance increments based on desired velocity
          distance_increment = cd.ctrl_speed * dd.delta_time;

          shift_x = 0.0;
          double next_x;
          double next_y;
          // calculate main car's trajectory points
          for (int i = 0; i < 30 - path_size; ++i)
          {
            // advance distance_increment meters down the road
            shift_x += distance_increment;
            // use spline to calculate y
            shift_y = spline(shift_x);
            // transform back to map coordinates
            next_x = shift_x * cos(angle) - shift_y * sin(angle);
            next_y = shift_x * sin(angle) + shift_y * cos(angle);
            next_x += pos_x;
            next_y += pos_y;
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

          nlohmann::json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
