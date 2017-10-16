#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen/Eigen/Core"
#include "Eigen/Eigen/QR"
#include "json.hpp"
#include "TrajectoryController.h"
#include "ChangingLaneRiskEvaluation.h"
using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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
	
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
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
	
	map_waypoints_x.push_back(map_waypoints_x[0]);
	map_waypoints_y.push_back(map_waypoints_y[0]);
	map_waypoints_s.push_back(max_s);
	map_waypoints_dx.push_back(map_waypoints_dx[0]);
	map_waypoints_dy.push_back(map_waypoints_dy[0]);
	map_waypoints_x.push_back(map_waypoints_x[1]);
	map_waypoints_y.push_back(map_waypoints_y[1]);
	map_waypoints_s.push_back(max_s+map_waypoints_s[1]);
	map_waypoints_dx.push_back(map_waypoints_dx[1]);
	map_waypoints_dy.push_back(map_waypoints_dy[1]);
    
    //Instantiate a Controller for our trajectories
	TrajectoryController TrajectoryController(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
    static int counter=0;

	h.onMessage([&TrajectoryController, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
                    
                    //Collect parameters for my car from the simulator
                    Vehicle myCar;
                
					// Main car's localization Data
					myCar.x = j[1]["x"];
					myCar.y = j[1]["y"];
					myCar.s = j[1]["s"];
					myCar.d = j[1]["d"];
					myCar.yaw = j[1]["yaw"];
					myCar.speed = j[1]["speed"];
                    
					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];
                    
                    //Collect more data on my car's current state
                    double  p_x;
                    double p_y;
                    if(counter == 0)
                    {
                        p_x = 0.0;
                        p_y = 0.0;;
                    }else{
                        p_x = previous_path_x.at(0);
                        p_y = previous_path_y.at(0);
                    }
                    double dx = myCar.x - p_x;
                    double dy = myCar.y - p_y;
                    double dt = 0.02;
                    myCar.vx = fabs(dx/dt);
                    myCar.vy = dy/dt;
                    
                    //Collect data on other cars from the simulator
					auto sensor_fusion = j[1]["sensor_fusion"];
					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;
                    
                    //Initialize my car state
					const std::vector<double> &car_state = {myCar.x, myCar.y,myCar.s,myCar.d,myCar.yaw,myCar.speed};
                    myCar.start_state = {myCar.x, myCar.y,myCar.s,myCar.d,myCar.yaw,myCar.speed};
                    
                    //Instantiate the risk evaluator
                    ChangingLaneRiskEvaluation risk_eval;
                    
                    //Evaluate the surrounding risks on each lane
                    vector<bool> no_go_lanes = risk_eval.EvaluateRisk(myCar, sensor_fusion);

                    //Call the controller
					TrajectoryController.generate_next_waypoints(car_state,previous_path_x,previous_path_y,end_path_s,end_path_d,sensor_fusion,no_go_lanes);
                    //Prepare the way points for the simulator from the path planned the controller
					next_x_vals = TrajectoryController.tc_next_x_vals;
					next_y_vals = TrajectoryController.tc_next_y_vals;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;
                    counter++;
					auto msg = "42[\"control\","+ msgJson.dump()+"]";
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
