/*
 * TrajectoryCost.h
 *
 */

#ifndef _TRAJECTORY_COST_H_
#define _TRAJECTORY_COST_H_

#include "vector"
#include "utils.h"


	double time_diff_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);
	double s_diff_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);

	double d_diff_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);

	double collision_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);
	double buffer_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);
	double stays_on_road_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);
	double exceeds_speed_limit_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);

	double total_accel_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);
	double max_accel_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);
	double total_jerk_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);
	double max_jerk_cost(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);

	double nearest_approach(const TrajectoryModel &traj, const Vehicle &vehicle);
	double nearest_approach_to_any_vehicle(const TrajectoryModel &traj, const std::map<int, Vehicle> &predictions);
	std::vector<double> differentiate(const std::vector<double> &coefficients);
	double to_equation(const std::vector<double> &coefficients, double t);
  
#endif /* _TRAJECTORY_COST_H_ */
