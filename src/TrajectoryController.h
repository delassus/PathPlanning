/*
 * TrajectoryController.h
 *
 */

#ifndef _TrajectoryController_H_
#define _TrajectoryController_H_

#include <vector>
#include "Trajectory.h"
#include "Control.h"

class TrajectoryController {
public:
	TrajectoryController(const vector<double> &maps_s, const vector<double> &maps_x, vector<double> &maps_y,
			const vector<double> &maps_dx, const vector<double> &maps_dy);
	void generate_next_waypoints(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
			const std::vector<std::vector<double>> &sensor_fusion, vector<bool> no_go_lanes);

	virtual ~TrajectoryController();
	std::vector<double> tc_next_x_vals;
	std::vector<double> tc_next_y_vals;
	vector<double> tc_map_s;
	vector<double> tc_map_x;
	vector<double> tc_map_y;
	vector<double> tc_map_dx;
	vector<double> tc_map_dy;
    vector<vector<double>> tc_last_waypoints_s;
    vector<vector<double>> tc_last_waypoints_d;
private:
	int previous_path_points;
    vector<double> getFrenet_Coordinates(double x, double y, double theta);
    TrajectoryCtrlStates checkRisk(int our_current_lane,int target_lane,vector<bool> no_go_lanes);
	int get_our_target_lane(TrajectoryCtrlStates suggested_state,int our_current_lane);
	std::vector<std::vector<double>> get_start_state(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d);
	std::map<int, Vehicle> get_predictions(const std::vector<std::vector<double>> &sensor_fusion, double start_s,const std::vector<double> &car_state);
	void adjust_next_waypoints(const TrajectoryModel &chosenTrajectory);
	Trajectory tc_trajectory;
    vector<double> convert_sd_to_xy(const double s, const double d);

	vector<double> getXY_Coordinates(double s, double d);
	
	std::vector<std::vector<double>> recycle_previous_path(const std::vector<double> &previous_path_x,
			const std::vector<double> &previous_path_y,double end_path_s,double end_path_d);
		Control tc_Control;
	TrajectoryCtrlStates tc_current_state;
    bool lane_change_manager(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
                             const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
                             const std::vector<std::vector<double>> &sensor_fusion);
};

#endif /* _TrajectoryController_H_ */
