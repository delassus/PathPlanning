/*
 * TrajectoryCtrlCost.h
 *
 */

#ifndef _TRAJECTORY_CONTROL_COST_H_
#define _TRAJECTORY_CONTROL_COST_H_

#include <map>
#include "utils.h"

enum TrajectoryCtrlStates {STAY_IN_LANE, GO_TO_LEFT_LANE, GO_TO_RIGHT_LANE};

int get_target_lane_id(const Vehicle & vehicle, TrajectoryCtrlStates state);
class TrajectoryCtrlCostData{
public:
	TrajectoryCtrlCostData(const Vehicle &vehicle_p, std::map<int, Vehicle> &predictions_p,
			std::map<int, Vehicle> cars_in_front_of_us_p,
			TrajectoryCtrlStates last_state_p, double time_since_last_change_p, int previous_lane_id_p): vehicle(vehicle_p), predictions(predictions_p)
    {
		cars_in_front_of_us = cars_in_front_of_us_p;
		last_state = last_state_p;
		time_since_last_change = time_since_last_change_p;
		previous_lane_id = previous_lane_id_p;
	}
    const Vehicle &vehicle;
	std::map<int, Vehicle> &predictions;
	
	std::map<int, Vehicle> cars_in_front_of_us;
	TrajectoryCtrlStates last_state;
	double time_since_last_change;
	int previous_lane_id;
};

double speed_cost(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data);
double collision_cost(const Vehicle & vehicle,  TrajectoryCtrlStates state, TrajectoryCtrlCostData &data);
double excessive_lane_change_cost(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data);
double lane_change_hysteresis_cost(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data);

#endif /* _TRAJECTORY_CONTROL_COST_H_ */
