#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
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


  

  enum TWIDDLE_STATES
  {
    TWIDDLE_INIT = 0,
    TWIDDLE_WAIT,
    TWIDDLE_ACCUMULATE_ERRORS,
    TWIDDLE_ADD_INIT,
    TWIDDLE_ADD_WAIT,
    TWIDDLE_ADD_ACCUMULATE_ERRORS,
    TWIDDLE_ADD_EVAL,
    TWIDDLE_SUBTRACT_INIT,
    TWIDDLE_SUBTRACT_WAIT,
    TWIDDLE_SUBTRACT_ACCUMULATE_ERRORS,
    TWIDDLE_SUBTRACT_EVAL,
    TWIDDLE_NEXT_PARAMETER

  };

  #define TWIDDLE_ITERATIONS 3000
  TWIDDLE_STATES state = TWIDDLE_INIT;
  bool twiddle_enabled = false;

  //double parameter[3] = {1.0, 0.0, 3.31};//3.05,0.01,20.5  //2.0, 0.10, 25.0
  //double parameter[3] = {0.134611, 0.000270736, 3.05349};//Initial: 0.15, 0.0, 3.71; Final: 0.35, 0.00811856, 4.08622
  //double parameter_delta[3] = {0.2, 0.004, 0.2};

  double parameter[3] = {0.4624, 0.000309193, 4.5}; //0.06, 0.00150, 3.0 //0.117156 : 0 : 4.5
  double parameter_delta[3] = {0.0, 0.0001, 0.0};

  double parameter_throttle[3] = {0.3, 0.0, 0.02};
  double error;
  double best_error;
  int count;
  int parameter_index;
  double throttle;
  
  //std::ofstream myfile;

int main()
{
  uWS::Hub h;

  PID pidSteering;
  PID pidThrottle;

  // TODO: Initialize the pid variable.
  pidSteering.Init(parameter[0], parameter[1], parameter[2]);
  pidThrottle.Init(parameter_throttle[0], parameter_throttle[1], parameter_throttle[2]);
  //myfile.open("data_points.txt");

  h.onMessage([&pidSteering, &pidThrottle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          if(twiddle_enabled)
          {
            switch(state)
            {
              case TWIDDLE_INIT:
              {
                count = 0;
                error = 0;
                parameter_index = 1;
                best_error = std::numeric_limits<double>::max();
                pidSteering.Init(parameter[0], parameter[1], parameter[2]);
                std::cout << "\nTWIDDLE INIT" << std::endl;
                std::cout << "Parameters: " << parameter[0] << " : " << parameter[1] <<" : " << parameter[2] << std::endl;
                std::cout << "DeltaParameters: " << parameter_delta[0] << " : " << parameter_delta[1] << " : " << parameter_delta[2] << std::endl;
                std::cout << "ERROR_0: " << error << ":" << best_error << std::endl;

                //Go to next state
                state = TWIDDLE_WAIT;
                break;
              }

              case TWIDDLE_WAIT: 
              {
                if(count <= 100) 
                {
                  pidSteering.UpdateError(cte);
                  steer_value = pidSteering.TotalError();
                  count += 1;
                }
                else
                {
                  count = 0;

                  //Go to next state
                  state = TWIDDLE_ACCUMULATE_ERRORS;
                }
                break;
              }

              case TWIDDLE_ACCUMULATE_ERRORS: 
              {
                if(count <= TWIDDLE_ITERATIONS)
                {
                  // Add cross track errors for each iteration
                  error += pow(cte, 2);
                  pidSteering.UpdateError(cte);
                  steer_value = pidSteering.TotalError();
                  count += 1;
                }
                else
                {
                  // Calculate the average error.
                  best_error = error / TWIDDLE_ITERATIONS;
                  std::cout << "ERROR_1: " << best_error << std::endl;
                  //Go to next state
                  state = TWIDDLE_ADD_INIT;
                }
                break;
              }

              case TWIDDLE_ADD_INIT:
              {
                count = 0;
                error = 0;
                parameter[parameter_index] += parameter_delta[parameter_index];
                pidSteering.Init(parameter[0], parameter[1], parameter[2]);
                pidSteering.Restart(ws);
                std::cout << "\nTWIDDLE ADD" << std::endl;
                std::cout << "Parameters: " << parameter[0] << " : " << parameter[1] <<" : " << parameter[2] << std::endl;
                std::cout << "DeltaParameters: " << parameter_delta[0] << " : " << parameter_delta[1] << " : " << parameter_delta[2] << std::endl;
                std::cout << "ERROR_0: " << error << ":" << best_error << std::endl;

                //Go to next state
                state = TWIDDLE_ADD_WAIT;
                break;
              }

              case TWIDDLE_ADD_WAIT: 
              {
                if(count <= 100) 
                {
                  pidSteering.UpdateError(cte);
                  steer_value = pidSteering.TotalError();
                  count += 1;
                }
                else
                {
                  count = 0;
                  //Go to next state
                  state = TWIDDLE_ADD_ACCUMULATE_ERRORS;
                }
                break;
              }

              case TWIDDLE_ADD_ACCUMULATE_ERRORS: 
              {
                if(count <= TWIDDLE_ITERATIONS)
                {
                  // Add cross track errors for each iteration
                  error += pow(cte, 2);
                  pidSteering.UpdateError(cte);
                  steer_value = pidSteering.TotalError();
                  count += 1;
                }
                else
                {
                  // Calculate the average error.
                  error /= TWIDDLE_ITERATIONS;
                  std::cout << "ERROR_1: " << error << ":" << best_error << std::endl;
                  //Go to next state
                  state = TWIDDLE_ADD_EVAL;
                }
                break;
              }

              case TWIDDLE_ADD_EVAL:
              {
                if(error < best_error)
                {
                  best_error = error;
                  parameter_delta[parameter_index] *= 1.1;

                  //Go to next state
                  state = TWIDDLE_NEXT_PARAMETER;
                }
                else
                {
                  //Go to next state
                  state = TWIDDLE_SUBTRACT_INIT;
                }
                break;
              }

              case TWIDDLE_SUBTRACT_INIT:
              {
                count = 0;
                error = 0;
                parameter[parameter_index] -= 2.0*parameter_delta[parameter_index];
                pidSteering.Init(parameter[0], parameter[1], parameter[2]);
                pidSteering.Restart(ws);
                std::cout << "\nTWIDDLE SUBTRACT" << std::endl;
                std::cout << "Parameters: " << parameter[0] << " : " << parameter[1] <<" : " << parameter[2] << std::endl;
                std::cout << "DeltaParameters: " << parameter_delta[0] << " : " << parameter_delta[1] << " : " << parameter_delta[2] << std::endl;
                std::cout << "ERROR_0: " << error << ":" << best_error << std::endl;

                //Go to next state
                state = TWIDDLE_SUBTRACT_WAIT;
                break;
              }

              case TWIDDLE_SUBTRACT_WAIT: 
              {
                if(count <= 100) 
                {
                  pidSteering.UpdateError(cte);
                  steer_value = pidSteering.TotalError();
                  count += 1;
                }
                else
                {
                  count = 0;
                  //Go to next state
                  state = TWIDDLE_SUBTRACT_ACCUMULATE_ERRORS;
                }
                break;
              }

              case TWIDDLE_SUBTRACT_ACCUMULATE_ERRORS: 
              {
                if(count <= TWIDDLE_ITERATIONS)
                {
                  // Add cross track errors for each iteration
                  error += pow(cte, 2);
                  pidSteering.UpdateError(cte);
                  steer_value = pidSteering.TotalError();
                  count += 1;
                }
                else
                {
                  // Calculate the average error.
                  error /= TWIDDLE_ITERATIONS;
                  std::cout << "ERROR_1: " << error << ":" << best_error << std::endl;
                  //Go to next state
                  state = TWIDDLE_SUBTRACT_EVAL;
                }
                break;
              }

              case TWIDDLE_SUBTRACT_EVAL:
              {
                if(error < best_error)
                {
                  best_error = error;
                  parameter_delta[parameter_index] *= 1.1;
                }
                else
                {
                  parameter[parameter_index] += parameter_delta[parameter_index];
                  parameter_delta[parameter_index] *= 0.9;
                }

                //Go to next state
                state = TWIDDLE_NEXT_PARAMETER;
                break;
              }
              case TWIDDLE_NEXT_PARAMETER:
              {
                std::cout << "\nTWIDDLE NEXT PARAMETER" << std::endl;
                std::cout << "Parameters: " << parameter[0] << " : " << parameter[1] <<" : " << parameter[2] << std::endl;
                std::cout << "DeltaParameters: " << parameter_delta[0] << " : " << parameter_delta[1] << " : " << parameter_delta[2] << std::endl;
                //Tune the next parameter
                parameter_index = 1; //(parameter_index + 1) % 3;
                //Go to next state
                state = TWIDDLE_ADD_INIT;

                break;
              }
            }


            pidThrottle.UpdateError(30.0-speed);
            throttle = pidThrottle.TotalError();

            if(throttle > 1.0)
            {
              throttle = 1.0;
            }
            if(throttle < -1.0)
            {
              throttle = -1.0;
            }

            if(steer_value > 1.0)
            {
              steer_value = 1.0;
            }
            if(steer_value < -1.0)
            {
              steer_value = -1.0;
            }
          }
          else
          {
            pidThrottle.UpdateError(30.0-speed);
            throttle = pidThrottle.TotalError();

            pidSteering.UpdateError(cte);
            steer_value = pidSteering.TotalError();


            if(throttle > 1.0)
            {
              throttle = 1.0;
            }
            if(throttle < -1.0)
            {
              throttle = -1.0;
            }

            if(steer_value > 1.0)
            {
              steer_value = 1.0;
            }
            if(steer_value < -1.0)
            {
              steer_value = -1.0;
            }

            /*if(myfile.is_open())
            {
              if(count >= 4000)
              {
                myfile.close();
              }
              else
              {
                myfile << cte << "," << steer_value << "," << speed << "\n";
                count += 1;
              }
            }*/
          }
          

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = -throttle; //0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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