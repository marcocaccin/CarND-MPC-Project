#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v *= 0.447; // Convert from [miles/h] to [m/s]
          double steering_angle = j[1]["steering_angle"]; // [rad]
          double throttle = j[1]["throttle"];
          // ******

          // Set the controller latency
          int latency_ms = 100;
          double latency_s = latency_ms / 1000.0;
          // A few useful values to carry around
          size_t len_pts = ptsx.size();
          double cos_psi = cos(psi);
          double sin_psi = sin(psi);

          // Vectors in local coordinate system
          Eigen::VectorXd ptsx_loc(len_pts);
          Eigen::VectorXd ptsy_loc(len_pts);
          double dx, dy;

          for (int i=0; i<len_pts; ++i) {
            dx = ptsx[i] - px;
            dy = ptsy[i] - py;
            // NOTE: rotation matrix is inverted here psi -> -psi
            ptsx_loc[i] = dx * cos_psi + dy * sin_psi;
            ptsy_loc[i] = -dx * sin_psi + dy * cos_psi;
          }

          // Fit cubic curve to waypoints
          auto coeffs = polyfit(ptsx_loc, ptsy_loc, 3);

          // Calculate the cross track error: value of polynomial fit at dt=0
          double cte = coeffs[0];

          // Calculate the orientation error: derivative of the fit at dt=0
          double epsi = -atan(coeffs[1]);

          // Predict future state at time dt=latency_s.
          // Assumption: longitudinal acceleration is zero, speed remains constant
          // In local reference frame, px, py, psi are 0
          double px_f = v * latency_s; // v*cos(psi)=v*cos(0)=v
          double py_f = 0.0; // v*sin(psi)=v*sin(0)=0
          double psi_f = - v * steering_angle / Lf * latency_s;
          // Is throttle the acceleration in m/s^2?
          // Not sure, let's keep speed constant then
          double v_f = v;
          double cte_f = cte + v * sin(epsi) * latency_s;
          double epsi_f = epsi + psi_f;

          // Create state vector at latency time
          Eigen::VectorXd state(6);
          state << px_f, py_f, psi_f, v_f, cte_f, epsi_f;

          // Update controller on the future state
          auto vars = mpc.Solve(state, coeffs);

          // get MPC control for steering angle and convert it in [-1, 1] range
          steering_angle = vars[0] / deg2rad(25);
          // get MPC control for throttle
          throttle = vars[1];

          // trajectory predicted by MPC
          vector<double> mpc_x, mpc_y;
          for (int i=2; i < vars.size(); ++i) {
              if (i % 2 == 0) {
                mpc_x.push_back(vars[i]);
              } else {
                mpc_y.push_back(vars[i]);
            }
          }

          // waypoints
          vector<double> next_x, next_y;
          for (int i = 0; i < len_pts; ++i) {
              next_x.push_back(ptsx_loc[i]);
              next_y.push_back(ptsy_loc[i]);
          }

          json msgJson;
          msgJson["steering_angle"] = steering_angle;
          msgJson["throttle"] = throttle;
          msgJson["mpc_x"] = mpc_x;
          msgJson["mpc_y"] = mpc_y;
          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          this_thread::sleep_for(chrono::milliseconds(latency_ms));
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
