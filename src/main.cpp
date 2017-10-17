#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <algorithm>
#include <stdlib.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  // check if we have something as an input to script
  double s_Kp, s_Ki, s_Kd, t_Kp, t_Ki, t_Kd, target_speed;

  // in case anything passed as parameters treat it as coefficients
  if (argc > 1) {
    s_Kp = atof(argv[1]);
    s_Ki = atof(argv[2]);
    s_Kd = atof(argv[3]);
    t_Kp = atof(argv[4]);
    t_Ki = atof(argv[5]);
    t_Kd = atof(argv[6]);
    target_speed = atof(argv[7]);

  // in other case set default coefficients. They are picked up to produce more or less appropriate result.
  } else {
    s_Kp = 0.08;
    s_Ki = 0.0003;
    s_Kd = 6.0;
    t_Kp = 0.3;
    t_Ki = 0.002;
    t_Kd = 0.5;
    target_speed = 20;
  }

  // declare PID controller for steer angle and throttle
  PID s_pid, t_pid;

  // steer angle controller
  s_pid.Init(s_Kp, s_Ki, s_Kd);

  // throttle controller
  t_pid.Init(t_Kp, t_Ki, t_Kd);

  h.onMessage([&s_pid, &t_pid, target_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          // commented as not utilised
//          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // define variable to calculate steer_value
          double steer_value;

          // update steer_value controller errors
          s_pid.UpdateError(cte);

          // calculate steer value here
          steer_value = -1 * s_pid.TotalError();

          // check that steering angle is between -1 and 1
          steer_value = fmin(steer_value, 1.0);
          steer_value = fmax(steer_value, -1.0);

          double throttle_value;

          // update throttle controller errors
          t_pid.UpdateError(target_speed - speed);
          throttle_value = t_pid.TotalError();

          std::cout << "CTE: " << cte << " Steering: " << steer_value << " Throttle: " << throttle_value << std::endl;

          // define responce message
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
