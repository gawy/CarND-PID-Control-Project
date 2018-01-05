#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <thread>
#include <chrono>

int ITERATION_LENGHT = 2000;
double ERR_THRESHOLD = 4.5*4.5;
double ERR_MAX = 10000.0;

double best_err;
int i_p;
double params[3] = {0.36, 0, 5.0}; //P I D params
double dt[3] = {.1, 0, 2};

long run_ctr = 0;
long ctr = 1;
int iteration_step = -1;


//forward declaration
//void run(uWS::Hub &h);
int setupUwsHub(uWS::Hub &h, PID &pid);

// for convenience

using json = nlohmann::json;
using namespace std;

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
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {


  PID pid;

  pid.Init(params[0], params[1], params[2]);

  uWS::Hub h;


  setupUwsHub(h, pid); //initialize and setup connection
  h.run();

}

int setupUwsHub(uWS::Hub &h, PID &pid) {
  std::cout<< "Socket setup finished" << std::endl;


  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {


    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      ctr++;
//      cout << ctr << "  " << endl;
      if (ctr % ITERATION_LENGHT == 0) {
        ctr = 0;
        run_ctr++;
        std::cout << "#" << run_ctr << std::endl;

        std::string reset_msg = "42[\"reset\",{}]";
        ws.send(reset_msg.data(), reset_msg.length(), uWS::TEXT);

        //re-init the pid controller
        double sum_dt = abs(dt[0]) + abs(dt[1]) + abs(dt[2]);
        if (sum_dt < 1e-10) {
          // time to exit
          std::cout << "We are done: " << dt[0] <<","<< dt[1] <<","<< dt[2] << std::endl;
          exit(0);
          return;
        }

        double err ; //to be intialized in every step where it is needed
        err = pid.RunError();
        cout << "Actual run error: " << err << endl;
        err = abs(err) > ERR_THRESHOLD ? ERR_MAX : err;

        //parameter iteration
        switch (iteration_step) {
          default:
            best_err = err;
            iteration_step = 0;

          case 0: //run 1 in the twiddle algorithm

            //just increment parameter and run
            params[i_p] += dt[i_p];
            iteration_step++;

            std::cout << "Step 0: i_p=" << i_p << ", params=" << params[0]<<","<< params[1]<<","<< params[2]
                      << "; err="<< err << ", dt="<< dt[0] << ", "<< dt[1] << ", "<< dt[2] << std::endl;
            break;
          case 1:
            std::cout << "Step 1: i_p=" << i_p << ", params=" << params[0]<<","<< params[1]<<","<< params[2]
                      << "; err="<< err << ", dt="<< dt[0] << ", "<< dt[1] << ", "<< dt[2] << std::endl;
            if (abs(err) < abs(best_err)) {
              std::cout << "Step 1, error reduced=" << err << std::endl;
              best_err = err;
              dt[i_p] *= 1.1;
              i_p = (i_p + (i_p == 0 ? 2:1)) % 3; //skip I tuning

              params[i_p] += dt[i_p];
              iteration_step = 1;
            } else {
              std::cout << "Step 1, error increased=" << err << std::endl;
              params[i_p] -= 2 * dt[i_p];
              iteration_step++;
            }


            break;
          case 2:
            std::cout << "Step 2: i_p=" << i_p << ", params=" << params[0]<<","<< params[1]<<","<< params[2]
                      << "; err="<< err << ", dt="<< dt[0] << ", "<< dt[1] << ", "<< dt[2] << std::endl;

            if (abs(err) < abs(best_err)) {
              std::cout << "Step 2, error reduced=" << err << std::endl;
              best_err = err;
              dt[i_p] *= -1.1;

            } else {
              std::cout << "Step 2, error increased=" << err << std::endl;
              params[i_p] += dt[i_p];
              dt[i_p] *= 0.9;
            }

            i_p = (i_p + (i_p == 0 ? 2:1)) % 3;//skip I tuning

            params[i_p] += dt[i_p];
            iteration_step = 1;
            break;
        }

        pid.Init(params[0], params[1], params[2]);

        return;
      }


      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
//          std::cout << j[1];
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
//          cout << "cte=" << cte << "; speed="<<speed << endl;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = -pid.TotalError();

          // DEBUG
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
//  h.run();

  return 0;
}


