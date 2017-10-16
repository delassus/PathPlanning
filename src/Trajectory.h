/*
 * Trajectory.h
 *
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include <map>
#include <iostream>
#include "utils.h"
using namespace std;

typedef  double (*CostFunPtr)(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);
class CostFuncWeight{
public:
	CostFuncWeight(){

	}
	CostFuncWeight(CostFunPtr cost_of_this, double weight_of_this){
		cost_func = cost_of_this;
		weight = weight_of_this;
	}
	CostFunPtr cost_func;
	double weight;
};

class Trajectory {
public:
	Trajectory();
	virtual ~Trajectory();
	TrajectoryModel keep_lane(const std::vector<double> &start_s, const std::vector<double> &start_d,
			double T, std::map<int, Vehicle> &predictions);
	std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);
	TrajectoryModel LANE_CHANGE(const std::vector<double> &start_s, const std::vector<double> &start_d,
			double T, std::map<int, Vehicle> &predictions, bool left= true);
	double calculate_cost(const TrajectoryModel &trajectory,  const std::map<int, Vehicle> &predictions);
	double tr_current_car_speed;


private:

	std::map<std::string, CostFuncWeight> c_cost;
	TrajectoryModel PTG(const std::vector<double> &start_s, const std::vector<double> &start_d,
			std::vector<TrajectoryModel> &all_trjs, double T,const std::map<int, Vehicle> &predictions);

	std::vector<TrajectoryModel> perturb_goals(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
			std::vector<double> &goal_s, std::vector<double> &goal_d,
			int target_vehicle, const std::vector<double> &delta, std::map<int, Vehicle> &predictions);
	TrajectoryModel pursue_goal(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
			std::vector<double> &goal_s, std::vector<double> &goal_d,  std::map<int, Vehicle> &predictions);
	TrajectoryModel follow_vehicle(const std::vector<double> &start_s, const std::vector<double> &start_d, double T,
			int target_vehicle, const std::vector<double> &delta,  std::map<int, Vehicle> &predictions);
};

#endif /* TRAJECTORY_H_ */
