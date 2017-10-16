/*
 * TrajectoryCtrlCost.cpp
 *
 */
#include<iostream>
#include <cmath>
#include "TrajectoryCtrlCost.h"
#include "utils.h"
#include "constants.h"

//This file contains functions that control trajectory cost and additional cost functions that are not trajectory related
//but behavioral

using namespace std;

/* Penalizes too frequent lane changing as it is not comfortable for passengers */
double excessive_lane_change_cost(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data){
    if(data.last_state == TrajectoryCtrlStates::STAY_IN_LANE)
    {
        bool change_request = (state != data.last_state);
        if(change_request){
            if(data.time_since_last_change < HYSTERESIS_COST) //lane changing is ok but only after some time
            {
                return 1;  //case not ok to change lane
            }
        }
    }
    
    return 0.0; //case ok to change lane
}

/* Penalizes collision that can be predicted (some cannot be predicted and avoided due to the weird suicidal behavior of some cars )*/

double collision_cost(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data)
{
    UTIL u;
    
    //If we stay in lane we assume no collision will happen
    if(state == TrajectoryCtrlStates::STAY_IN_LANE)
    {
        return 0.0;
    }
    
    int target_lane_id = get_target_lane_id(vehicle, state);
    double gap_in_front_of_us = INFINITY;
    double gap_behind_us = INFINITY;
    
    //Find if on the target lane there is a gap with no other cars in front of us
    //(1) Are there cars on that lane in front of us?
    if(data.cars_in_front_of_us.find(target_lane_id) != data.cars_in_front_of_us.end())
    {
        //(2) If there are cars on that lane what is the distance gap with us?
        gap_in_front_of_us = data.cars_in_front_of_us[target_lane_id].start_state[0] - vehicle.start_state[0];
    }
    
    //Check if there is a similar gap behind us
    for (const auto& measured : data.predictions)
    {
        const Vehicle &v = measured.second;
        //If there are no other cars on the target lane skip
        if(u.which_lane(v.start_state[3]) != target_lane_id)
        {
            continue;
        }
        //if the other car is not behind us, skip
        if(v.start_state[0] > vehicle.start_state[0])
        {
            continue;
        }
        //Measure the distance with us
        double gap = vehicle.start_state[0] - v.start_state[0];
        if(gap < gap_behind_us)
        {
            //Find the minimum distance gap
            gap_behind_us = gap;
        }
    }
    //If both the front gap and the rear gap are lage enough do not penalize
    if(gap_in_front_of_us > FRONT_SECURITY_BUFFER && gap_behind_us> BACK_SECURITY_BUFFER)
    {
        return 0.0;
    }
    
    //if either the front gap or the rear gap are small penalize
    return 1.0;
}

/* Penalizes toggling between the same two lanes */
double lane_change_hysteresis_cost(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data)
{
    
    if(data.last_state == TrajectoryCtrlStates::GO_TO_LEFT_LANE || data.last_state == TrajectoryCtrlStates::GO_TO_RIGHT_LANE)
    {
        int target_lane_id = get_target_lane_id(vehicle, state);
        if(target_lane_id != data.previous_lane_id)
        {
            return 1.0;
        }
    }
    return 0.0;
}

/* Computes the target lane from the car frenet coordinate d */
int get_target_lane_id(const Vehicle & vehicle, TrajectoryCtrlStates state)
{
    UTIL u;
	int current_lane = u.which_lane(vehicle.start_state[3]);
	int target_lane_id = current_lane;
	if(state == TrajectoryCtrlStates::GO_TO_LEFT_LANE)
    {
		target_lane_id = current_lane - 1;
	}else if (state == TrajectoryCtrlStates::GO_TO_RIGHT_LANE)
    {
		target_lane_id = current_lane +1;
	}
	return target_lane_id;
}

/* Penalizes selecting a lane with slow vehicles as target lane */
double speed_cost(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data)
{
    UTIL u;
	int target_lane_id = get_target_lane_id(vehicle, state);
	double target_lane_speed = target_lane_speed = MAX_METERS_PER_SECOND_SPEED + 10.0;

	if(data.cars_in_front_of_us.find(target_lane_id) != data.cars_in_front_of_us.end())
    {
		target_lane_speed = data.cars_in_front_of_us[target_lane_id].start_state[1];
	}
	return u.logistic(MAX_METERS_PER_SECOND_SPEED/target_lane_speed);
}

