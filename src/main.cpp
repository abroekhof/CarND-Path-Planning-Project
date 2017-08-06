#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"

using namespace std;
using namespace Eigen;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int d_to_lane(double d)
{
	int lane = floor(d / 4);
	lane = min(2, lane);
	lane = max(0, lane);
	return lane;
}

vector<double> JMT(vector<double> start, vector<double> end, double T)
{
	Matrix3d A;
	A << T * T * T, T * T * T * T, T * T * T * T * T,
		3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
		6 * T, 12 * T * T, 20 * T * T * T;

	Vector3d B;
	B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
		end[1] - (start[1] + start[2] * T),
		end[2] - start[2];

	Matrix3d Ai = A.inverse();

	Vector3d C = Ai * B;

	vector<double> result = {start[0], start[1], .5 * start[2]};
	for (int i = 0; i < C.size(); i++)
	{
		result.push_back(C.data()[i]);
	}

	return result;
}

double polyeval(vector<double> const &coeff, double t)
{
	double t_pow = 1.0;
	double result = 0.0;
	for (int i = 0; i < coeff.size(); ++i)
	{
		result += t_pow * coeff[i];
		t_pow *= t;
	}
	return result;
}

int main()
{
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

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
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
	// Add the first point again so the spline wraps around
	map_waypoints_x.push_back(map_waypoints_x[0]);
	map_waypoints_y.push_back(map_waypoints_y[0]);
	map_waypoints_s.push_back(map_waypoints_s[0] + max_s);
	map_waypoints_dx.push_back(map_waypoints_dx[0]);
	map_waypoints_dy.push_back(map_waypoints_dy[0]);

	tk::spline s_x, s_y, s_dx, s_dy;
	s_x.set_points(map_waypoints_s, map_waypoints_x);
	s_y.set_points(map_waypoints_s, map_waypoints_y);
	s_dx.set_points(map_waypoints_s, map_waypoints_dx);
	s_dy.set_points(map_waypoints_s, map_waypoints_dy);

	double last_s = 0.0;
	double curr_inc = 0.0;
	double last_d = 6.0;

	bool changing_d = false;
	vector<double> d_coeff;
	double d_change_time = 0.0;

	h.onMessage([max_s, &s_x, &s_y, &s_dx, &s_dy, &last_s, &curr_inc, &last_d, &d_coeff, &d_change_time, &changing_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
																													  uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

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

					int curr_lane = d_to_lane(car_d);
					int target_lane = curr_lane;
					double target_d = target_lane * 4 + 2;

					double target_inc = 46 * 1.60934 / 3600 * 1000 / 50;

					double lane_speeds[3] = {target_inc, target_inc, target_inc};

					// Scan through all cars
					for (int i = 0; i < sensor_fusion.size(); ++i)
					{
						double sd_vx = sensor_fusion[i][3];
						double sd_vy = sensor_fusion[i][4];
						double mag = sqrt(sd_vx * sd_vx + sd_vy * sd_vy);

						double sf_s = sensor_fusion[i][5];
						double sf_d = sensor_fusion[i][6];
						int sf_lane = d_to_lane(sf_d);

						double diff_s = sf_s - last_s;
						double diff_s_car = sf_s - car_s;
						double diff_d = sf_d - last_d;

						// How fast are the lanes going?
						if (diff_s < 10 && diff_s_car > -5)
						{
							lane_speeds[sf_lane] = min(lane_speeds[sf_lane], mag / 50);
							// Slow down if too close to the followed car.
							if (curr_lane == sf_lane && diff_s_car < 10)
							{
								std::cout << "Following too close!" << std::endl;
								lane_speeds[sf_lane] -= 5;
							}
						}
						// Are there cars to the side?
						if (abs(diff_s_car) < 15 && curr_lane != sf_lane)
						{
							lane_speeds[sf_lane] = 0.0;
						}
					}

					double max_speed = lane_speeds[curr_lane];
					// Choose the best lane
					for (int i = 0; i < 3; ++i)
					{
						// Can switch two lanes iff middle lane is open.
						if ((abs(curr_lane - i) == 1 || lane_speeds[1] > 0) && lane_speeds[i] > (max_speed * 1))
						{
							target_lane = i;
							max_speed = lane_speeds[i];
						}
					}
					// Start a lane change!
					if (target_lane != curr_lane && !changing_d)
					{
						std::cout << "\n New target lane: " << target_lane << std::endl;
						target_d = target_lane * 4 + 2;
						changing_d = true;
						d_change_time = 0.0;

						d_coeff = JMT({last_d, 0, 0}, {target_d, 0, 0}, 3);
					}
					target_inc = max_speed;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					int path_size = previous_path_x.size();
					for (int i = 0; i < path_size; i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}
					// Acceleration value
					double inc_diff = .001;
					for (int i = 0; i < 50 - path_size; i++)
					{
						// Speed control:
						if (curr_inc > target_inc)
						{
							curr_inc -= inc_diff;
						}
						else
						{
							curr_inc += inc_diff;
						}

						last_s = (last_s + curr_inc);
						// Wrap around the track
						if (last_s > max_s)
						{
							last_s = fmod(last_s, max_s);
						}

						double dx = s_dx(last_s);
						double dy = s_dy(last_s);
						double mag = sqrt(dx * dx + dy * dy);
						dx /= mag;
						dy /= mag;

						if (changing_d)
						{
							d_change_time += 1.0 / 50.0;
							last_d = polyeval(d_coeff, d_change_time);
							// Stop changing lanes after the desired amount of time has passed.
							changing_d = d_change_time <= 3;
						}
						std::cout << "s: " << last_s << ", d: " << last_d << ", target_d: " << target_d << ", changing_d: " << changing_d << ", d_change_time: " << d_change_time << std::endl;
						double new_x = s_x(last_s) + last_d * dx;
						double new_y = s_y(last_s) + last_d * dy;

						next_x_vals.push_back(new_x);
						next_y_vals.push_back(new_y);
					}

					json msgJson;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
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
