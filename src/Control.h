/*
 * Control.h
 *
 */

#ifndef _Control_H_
#define _Control_H_

#include <string>
#include <map>
#include <vector>
#include "TrajectoryCtrlCost.h"
#include "utils.h"

typedef  double (*TrajectoryCtrlCostFunPtr)(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data);

class TrajectoryCtrlCostWeight{
public:
    TrajectoryCtrlCostFunPtr cost_func;
    double weight;
	TrajectoryCtrlCostWeight(){

	}
	TrajectoryCtrlCostWeight(TrajectoryCtrlCostFunPtr cost_of_this, double weight_of_this){
		cost_func = cost_of_this;
		weight = weight_of_this;
	}
	
};


class Control {
public:
	Control();
	virtual ~Control();
	TrajectoryCtrlStates update_state(const std::vector<double> &start_s, const std::vector<double> &start_d,
			std::map<int, Vehicle> &predictions);
private:
    int previous_lane_used;
	double calculate_cost(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data);
	std::map<std::string, TrajectoryCtrlCostWeight> c_cost;
	TrajectoryCtrlCostData compute_TrajectoryCtrl_cost_data(const Vehicle & vehicle, std::map<int, Vehicle> &predictions);
    vector<TrajectoryCtrlStates> get_all_states(const std::vector<double> &start_d);
    TaskTimer Timer;
    TrajectoryCtrlStates tc_last_state;
};

#endif /* _Control_H_ */
