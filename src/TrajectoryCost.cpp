/*
 * TrajectoryCost.cpp
 *
 */

#include "TrajectoryCost.h"
#include <iostream>
#include <cmath>
#include "constants.h"
#include <algorithm>
#include "TrajectoryController.h"

using namespace std;

//These functions have been ported to c++ from Python
//They can be found in Udacity's helper file

/*
   Penalizes trajectories that span a duration which is longer or
   shorter than the duration requested.
 */
double time_diff_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions)
{
    UTIL u;
	double t = traj.t;
	double plain_trajectory = traj.plain_trajectory;

	return u.logistic(float(abs(t-plain_trajectory)) / plain_trajectory);
}


/*
   Penalizes trajectories when s coordinate (and derivatives)
    differ from the goal.
 */
double s_diff_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions)
{
    UTIL u;
	const vector<double> &s_coeff = traj.s_coeff;
	const vector<double> &plain_s = traj.plain_s;
	double t = traj.t;

	const vector<double> s = s_coeff;
	const vector<double> s_dot = differentiate(s);
	const vector<double> s_dot_dot = differentiate(s_dot);

	vector<double> s_actual = {to_equation(s, t),to_equation(s_dot, t),to_equation(s_dot_dot, t)};

	double cost = 0;
	for (int i =0; i< plain_s.size(); i++){
		double actual = s_actual[i];
		double expected = plain_s[i];
		double sigma = SIGMA_S[i];

		double diff = float(abs(actual-expected));
		cost += u.logistic(diff/sigma);
	}
	return cost;
}
/*
 Calculates the derivative of a polynomial and returns the corresponding coefficients.
 */
std::vector<double> differentiate(const std::vector<double> &coefficients){
	std::vector<double> new_cos = {};
	for (int i=0; i<coefficients.size();i++ ){
		if (i==0){
			continue;
		}
		new_cos.push_back(i * coefficients[i]);
	}
	return new_cos;
}
/*
 Takes the coefficients of a polynomial and point t, calculate the f value
 */
double to_equation(const std::vector<double> &coefficients, double t){
	double total = 0.0;
	for (int i = 0; i < coefficients.size(); i++ ){
		total += coefficients[i] * pow(t, i);
	}
	return total;
}


/*
   Penalizes trajectories when d coordinate (and derivatives)
    differ from the goal.
 */
double d_diff_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions)
{
    UTIL u;
	const vector<double> &d_coeff = traj.d_coeff;
	const vector<double> &plain_d = traj.plain_d;
	double t = traj.t;

	const vector<double> d = d_coeff;
	const vector<double> d_dot = differentiate(d);
	const vector<double> d_dot_dot = differentiate(d_dot);

	vector<double> d_actual = {to_equation(d, t),to_equation(d_dot, t),to_equation(d_dot_dot, t)};

	double cost = 0;
	for (int k = 0; k< plain_d.size(); k++){
		double actual = d_actual[k];
		double expected = plain_d[k];
		double sigma = SIGMA_D[k];

		double diff = float(abs(actual-expected));
		cost += u.logistic(diff/sigma);
	}
	return cost;
}

double nearest_approach(const TrajectoryModel &traj, const Vehicle &vehicle){
	double closest = INFINITY;
	const vector<double> &s_coeffs = traj.s_coeff;
	const vector<double> &d_coeffs = traj.d_coeff;
	double t = traj.t;


	for(int i=0; i< 100; i++){
		double cur_t = float(i) / 100 * t;
		double cur_s = to_equation(s_coeffs, cur_t);
		double cur_d = to_equation(d_coeffs, cur_t);
		std::vector<double> v_state = vehicle.state_in(cur_t);

		double targ_s = v_state[0];
		double targ_d = v_state[3];
		double dist = sqrt(((cur_s - targ_s) * (cur_s - targ_s)) + ((cur_d - targ_d) * (cur_d-targ_d)));
		if(dist < closest){
			closest = dist;
		}
	}
	return closest;

}
/*
  Calculates the closest distance to any vehicle during a trajectory.
 */
double nearest_approach_to_any_vehicle(const TrajectoryModel &traj, const std::map<int,
		Vehicle> &predictions)
{
    UTIL u;
	double closest = INFINITY;
	int closest_vehicle_id = -1;
	for (const auto& measured : predictions) {
		if(u.which_lane(measured.second.start_state[3]) == u.which_lane(traj.d_coeff[0]) && measured.second.start_state[0] < traj.s_coeff[0])
        {
			continue;
		}
		double d = nearest_approach(traj, measured.second);
		if(d < closest){
			closest = d;
			closest_vehicle_id = measured.first;
		}
	}
	
	return closest;

}
/*
 *  Cost function which penalizes collisions.
 */
double collision_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions)
{
	double nearest = nearest_approach_to_any_vehicle(traj, predictions);
	if(nearest < 2*VEHICLE_RADIUS)
    {
        return 1.0;
	}

	else
		return 0.0;
}
/*
 * Penalizes getting close to other vehicles.
 */

double buffer_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions)
{
    UTIL u;
	double nearest = nearest_approach_to_any_vehicle(traj, predictions);
	return u.logistic(2*VEHICLE_RADIUS / nearest);
}

/* Penalizes if we are not on the road anymore */
double stays_on_road_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions){
	const vector<double> &d_coeffs = traj.d_coeff;
	double t = traj.t;
	vector<double> all_ds = {};
	for(int i=0; i< 100;i++){
		double cur_t = float(t)/100 * i;
		all_ds.push_back(to_equation(d_coeffs, cur_t));
	}

	auto max_d = std::max_element(std::begin(all_ds), std::end(all_ds));
	auto min_d = std::min_element(std::begin(all_ds), std::end(all_ds));
    
	if(*max_d <= MAX_D && *min_d >=MIN_D){

		return 0;
	}
	
	return 1;
}
/* Penalizes exceeding the  speed limit */
double exceeds_speed_limit_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions){

	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);
	double t = traj.t;
	vector<double> all_vs = {};
	for(int i=0; i< 100;i++){
		double cur_t = float(t)/100 * i;
		all_vs.push_back(to_equation(s_dot_coeffs, cur_t));
	}

	auto max_v = std::max_element(std::begin(all_vs), std::end(all_vs));
	auto min_v = std::min_element(std::begin(all_vs), std::end(all_vs));

	if(*max_v <= MAX_METERS_PER_SECOND_SPEED && *min_v >= MIN_SPEED){
		return 0;
	}
	
	return 1;
}
/* Penalizes too much acceleration over the trajectory*/
double total_accel_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions)
{
    UTIL u;
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);

	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);
	double t = traj.t;

	double dt = t / 100.0;

	double cur_t= 0;
	double total_acc = 0;

	for(int i=0; i< 100; i++){
		cur_t = dt * i;
		double acc = to_equation(s_dot_dot_coeffs, cur_t);
		total_acc += abs(acc*dt);
	}

	double acc_per_second = total_acc / cur_t;

	return u.logistic(acc_per_second / EXPECTED_ACC_PER_SECOND );

}

/* Penalizes excessive instant acceleration */
double max_accel_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions){
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);

	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);
	double t = traj.t;

	vector<double> all_as = {};
	for(int i=0; i< 100;i++){
		double cur_t = float(t)/100 * i;
		all_as.push_back(to_equation(s_dot_dot_coeffs, cur_t));
	}

	auto max_acc = std::max_element(std::begin(all_as), std::end(all_as));
	
	if (abs(*max_acc) > MAX_ACCEL)
    {
		return 1;
	}else{
		return 0;
	}
}
/* Penalizes too much jerk over the trajectory */
double total_jerk_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions)
{
    UTIL u;
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);

	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);

	vector<double> s_dot_dot_dot_coeffs = differentiate(s_dot_dot_coeffs);
	double t = traj.t;

	double dt = t / 100.0;

	double cur_t= 0;
	double total_jerks = 0;

	for(int i=0; i< 100; i++){
		cur_t = dt * i;
		double jerk = to_equation(s_dot_dot_dot_coeffs, cur_t);
		total_jerks += abs(jerk*dt);
	}
	double jerk_per_second = total_jerks / cur_t;

	return u.logistic(jerk_per_second / EXPECTED_JERK_PER_SECOND );

}
/* Penalizes too much instant jerk */
double max_jerk_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions)
{
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);

	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);

	vector<double> s_dot_dot_dot_coeffs = differentiate(s_dot_dot_coeffs);
	double t = traj.t;

	vector<double> all_jerks = {};
	for(int i=0; i< 100;i++){
		double cur_t = float(t)/100 * i;
		all_jerks.push_back(to_equation(s_dot_dot_dot_coeffs, cur_t));
	}

	auto max_jerk = std::max_element(std::begin(all_jerks), std::end(all_jerks));
	
	if (abs(*max_jerk) > MAXIMUM_JERK_ALLOWED){
        return 1;
	}else{
		return 0;
	}
}
