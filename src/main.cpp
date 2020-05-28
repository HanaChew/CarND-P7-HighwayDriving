#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

//#include <cstdio>
//#include <cstdlib>


// for convenience
using nlohmann::json;
using std::string;
using std::vector;




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

  // assume starting velocity is 0 and startLane is 1
  double currVel = 0.0;
  int lane = 1;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
    &map_waypoints_dx,&map_waypoints_dy,
    &lane, &currVel]
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

              json msgJson;

              int prevSize = previous_path_x.size();


              /*************************************************************
              *  check for cars in the neighbouring lanes up to 30m ahead  *
              **************************************************************/
              if (prevSize > 0) {
                car_s = end_path_s;
              }

              bool tooCloseFront = false;
              bool carOnLeft = false;
              bool carOnRight = false;

              // check for each car i present in sensor fusion data
              for (int i=0; i<sensor_fusion.size(); i++){
                float d = sensor_fusion[i][6];
                if (d > (4*lane) && d < (4 + 4*lane) ){
                  tooCloseFront =  tooCloseFront || getTooClose(sensor_fusion[i], car_s, prevSize);
                }
                // check left lane
                if (lane !=0 && d > (4*lane -4) && d < (4*lane) ){
                  carOnLeft =  carOnLeft || getTooClose(sensor_fusion[i], car_s, prevSize);
                }
                // check right lane
                if (lane !=2 && d > (4*lane+4) && d < (4*lane+8) ){
                  carOnRight = carOnRight || getTooClose(sensor_fusion[i], car_s, prevSize);
                }
              }

              /***************************************************************
              *  defining a path made up of (x,y) points that the car visits *
              *  sequentially every .02 seconds                              *
              ****************************************************************/

              vector<double> trajectoryX;
              vector<double> trajectoryY;

              double refX = car_x;
              double refY = car_y;

              int numWPBehindSpline = 4;
              int numWPAheadSpline = 4;

              // if prevSize is almost empty, use car as startRef and
              // extrapolate 1 waypoint into the past
              if ( prevSize < numWPBehindSpline ) {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                trajectoryX.push_back(prev_car_x);
                trajectoryY.push_back(prev_car_y);
                trajectoryX.push_back(car_x);
                trajectoryY.push_back(car_y);
              } else {
                for (int j=numWPBehindSpline; j>0; j--){
                  refX = previous_path_x[prevSize - j];
                  refY = previous_path_y[prevSize - j];
                  trajectoryX.push_back(refX);
                  trajectoryY.push_back(refY);
                }
              }

              // Setting up future waypoints to be used in the trajectory
              for (int j=1; j<=numWPAheadSpline; j++){
                vector<double> futureWaypoint = getXY(car_s + 30*j, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                trajectoryX.push_back(futureWaypoint[0]);
                trajectoryY.push_back(futureWaypoint[1]);
              }

              double carYawRad = deg2rad(car_yaw);

              // Transform global coordinates to local car coordinates.
              for ( int i = 0; i < trajectoryX.size(); i++ ) {
                double xOffset = trajectoryX[i] - refX;
                double yOffset = trajectoryY[i] - refY;

                trajectoryX[i] = xOffset * cos(-carYawRad) - yOffset * sin(-carYawRad);
                trajectoryY[i] = xOffset * sin(-carYawRad) + yOffset * cos(-carYawRad);
              }


              tk::spline s;
              s.set_points(trajectoryX, trajectoryY);

              vector<double> next_x_vals;
              vector<double> next_y_vals;

              for ( int i = 0; i < prevSize; i++ ) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              // Calculate y position 30 m ahead of current position.
              double target_x = 30.0;
              double target_y = s(target_x);
              double target_dist = sqrt(target_x*target_x + target_y*target_y);

              double x_add_on = 0;

              // define finer points on the trajectory generated
              for( int i = 1; i < 50 - prevSize; i++ ) {
                // if car too close to front first try changing lanes if there is space
                if (tooCloseFront){
                  // check for cars on the left lane, & switch to that lane
                  // provided ego-car is not on left-most lane
                  if (lane>0 && !carOnLeft) {
                    lane -= 1;
                  }
                  // if not possible, check for cars on the right lane, & switch
                  // to that lane provided ego-car is not on right-most lane
                  else if (lane <2 && !carOnRight){
                    lane += 1;
                  } else {
                    // if changing lanes is not possible, slow down!
                    currVel -= 0.224; // acc = 10ms2 -0.448
                  }
                } else if  (currVel < 49) { // speed up to speed limit if free
                  currVel += 0.224; // acc = 10ms2 -0.448
                }

                double N = target_dist/(0.02*currVel/2.24);
                double x_point = x_add_on + target_x/N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                x_point = x_ref * cos(carYawRad) - y_ref * sin(carYawRad);
                y_point = x_ref * sin(carYawRad) + y_ref * cos(carYawRad);

                x_point += refX;
                y_point += refY;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
              }


              /***************************************************************
              *  end of edited code                                          *
              ****************************************************************/


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
