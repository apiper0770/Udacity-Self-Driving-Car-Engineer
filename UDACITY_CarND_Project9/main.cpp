#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

int main()
{
  uWS::Hub h;

  PID pid;
  //Initialize the pid variable.
  //Manually tuned parameters. Utilized the follwoing website to manually tune PID.
  //https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
  //Basically started with P control to get stable oscilations, car would go off track at turns
  //Then went to D control to get car to hone into the turn parameter faster, so it could make turns effectively
  //Then I increased P value and D values slightly until the car drove pretty well around the track.
  //Finally i adjusted the I value, so that the car would correct for being on one side of the track for too long..
  //After testing P value should be around 0.1
  //After testing with D value, it should be around 1.35
  //Kfter testing with I value, it should be around 0.002
  pid.Init(0.1, 0.002, 1.35);//final
//  pid.Init(0.1, 0.0, 0.0); //only P
//  pid.Init(0.1, 0.0, 1.35);// P & D

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          //double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
/*
           //start of twiddle algorithm
    double tol = 0.05;
    p = [0, 0, 0];
    dp = [1, 1, 1];
    robot = make_robot();
    x_trajectory, y_trajectory, best_err = run(robot, p);
    //twiddle loop here
    it = 0;
    while (sum(dp) > tol){
        print("Iteration {}, best error = {}".format(it, best_err))
        for(int i = 0; i < len(p); i++){
            p[i] += dp[i];
            robot = make_robot();
            x_trajectory, y_trajectory, err = run(robot, p);

            if(err < best_err){
                best_err = err;
                dp[i] *= 1.1;
            }
            else{
                p[i] -= 2 * dp[i];
                robot = make_robot();
                x_trajectory, y_trajectory, err = run(robot, p);

                if(err < best_err){
                    best_err = err;
                    dp[i] *= 1.1;
                }else{
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
        }
        }
        it += 1
    }
    return p, best_err
           
           */
          //update error before calculating steer_value
          pid.UpdateError(cte);

          //calculate new steer_value
          steer_value = -pid.TotalError();
          if(steer_value > 1){
              steer_value = 1.0;
          }else if(steer_value < -1){
              steer_value = -1.0;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
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
