#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"

#include "tracking.h"
#include "tools.h"
#include "models/ctv.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string GetData(const string& s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return string();
  }
  // cout << s << endl;
  if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return string();
}

int main()
{
  uWS::Hub h;

  Tracking<CTV::Model> fusion;
  Tools::RMSE RMSE;
  // By default, all sensors are enabled.
  // Uncomment this line to enable only LIDAR measurements.
  // fusion.sensors_ = MeasurementPackage::LASER;

  h.onMessage([&fusion, &RMSE](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (!(length > 2 && data[0] == '4' && data[1] == '2')) {
      return;
    }

    auto s = GetData(string(data, length));
    if (s.empty()) {
      string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      return;
    }
    // cout << s << endl;

    auto j = json::parse(s);
    string event = j[0].get<string>();
    if (event != "telemetry") {
      return;
    }
    
    // j[1] is the data JSON object
    string sensor_measurment = j[1]["sensor_measurement"];
    istringstream iss(sensor_measurment);

    // reads first element from the current line
    string sensor_type;
    iss >> sensor_type;

    MeasurementPackage measurement;
    if (sensor_type.compare("L") == 0) {
      measurement.sensor_type_ = MeasurementPackage::LASER;
      measurement.raw_measurements_ = VectorXd(2);
      float px, py;
      iss >> px >> py;
      measurement.raw_measurements_ << px, py;
    } else if (sensor_type.compare("R") == 0) {
      measurement.sensor_type_ = MeasurementPackage::RADAR;
      measurement.raw_measurements_ = VectorXd(3);
      float ro, phi, ro_dot;
      iss >> ro >> phi >> ro_dot;
      measurement.raw_measurements_ << ro, phi, ro_dot;
    }
    iss >> measurement.timestamp_;
    float x_gt, y_gt, vx_gt, vy_gt;
    iss >> x_gt >> y_gt >> vx_gt >> vy_gt;

    VectorXd ground_truth(4);
    ground_truth << x_gt, y_gt, vx_gt, vy_gt;

    fusion.Process(measurement);
    VectorXd estimate = fusion.GetEstimate();
    VectorXd rmse = RMSE.Update(estimate, ground_truth);

    // Push the current estimated (x, y) position
    json msgJson;
    msgJson["estimate_x"] = estimate(0);
    msgJson["estimate_y"] = estimate(1);
    msgJson["rmse_x"] =  rmse(0);
    msgJson["rmse_y"] =  rmse(1);
    msgJson["rmse_vx"] = rmse(2);
    msgJson["rmse_vy"] = rmse(3);
    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
    // cout << msg << endl;
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    // cout << "Disconnected" << endl;
    ws.close();
    cout << "Disconnected." << endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    cout << "Listening to port " << port << endl;
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
