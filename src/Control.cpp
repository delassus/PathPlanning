/*
 * Control.cpp
 *
 */

#include "Control.h"
#include <cmath>
#include<iostream>
#include <algorithm>
#include "utils.h"

using namespace std;

/* Constructor */
Control::Control() {
	c_cost["excessive_lane_change_cost"] = TrajectoryCtrlCostWeight(&excessive_lane_change_cost, 10);
	c_cost["lane_change_hysteresis_cost"] = TrajectoryCtrlCostWeight(&lane_change_hysteresis_cost, 100);
    c_cost["speed_cost"] = TrajectoryCtrlCostWeight(&speed_cost, 1);
    c_cost["collision_cost"] = TrajectoryCtrlCostWeight(&collision_cost, 10);
    
	tc_last_state = TrajectoryCtrlStates::STAY_IN_LANE;
	previous_lane_used = 1;
}
/* Destructor */
Control::~Control() {


}


/* STATE MACHINE FOR MONITORING THE CAR PATH PLANNING
There are three states: STAY_IN_LANE, GOTO_LEFT_LANE, GOTO_RIGHT_LANE.
States GOTO_RIGHT_LANE and GOTO_LEFT_LANE are not cancellable : once initiated they will go to completion.
Completion occurs when the car reaches the target lane center. This usually takes three seconds.
This feature of "uncancellable command" to change lane was introduced to avoid "out of lane" errors by the simulator
 if the car takes too long in between lanes.
The car would spend too much time between lanes if a change lane is initiated and then rolled back before completion.
Because "change lane" is a command that cannot be cancelled, careful study of the car surroundings is necessary before
 giving the command to change lane. In particular, cars speeding close behind us in adjacent lanes have to be detected.
 Even with all these  precautions many unavoidable collisions will happen because some cars are not behaving prudently.
 We will find cars just rushing to collide us from the side and the rear. 
 Unavoidable collision rate between us and another crazy driver seems to be 2 an hour.
 In the same time span of one hour we will witness at least three collisions between other cars.
 
 */
 

vector<TrajectoryCtrlStates> Control::get_all_states(const std::vector<double> &start_d){
    UTIL u;
	vector<TrajectoryCtrlStates> states = {};
	int current_lane = u.which_lane(start_d[0]);
    
    /* set what future states become now potentially reachable given our last state */
	switch(tc_last_state){
	case TrajectoryCtrlStates::STAY_IN_LANE:
		states = {TrajectoryCtrlStates::STAY_IN_LANE, TrajectoryCtrlStates::GO_TO_LEFT_LANE, TrajectoryCtrlStates::GO_TO_RIGHT_LANE};
		break;
	case TrajectoryCtrlStates::GO_TO_LEFT_LANE:
		states = {TrajectoryCtrlStates::STAY_IN_LANE, TrajectoryCtrlStates::GO_TO_LEFT_LANE};
		break;
	case TrajectoryCtrlStates::GO_TO_RIGHT_LANE:
		states = {TrajectoryCtrlStates::STAY_IN_LANE, TrajectoryCtrlStates::GO_TO_RIGHT_LANE};
		break;
	}
	
/* House keeping: in the following we remove states that are not reachable due to our current lane */
	switch(current_lane){
	case 2:
		//remove GO_TO_RIGHT_LANE
		states.erase(std::remove(states.begin(), states.end(), TrajectoryCtrlStates::GO_TO_RIGHT_LANE), states.end());
		break;
	case 0:
		//remove GO_TO_LEFT_LANE
		states.erase(std::remove(states.begin(), states.end(), TrajectoryCtrlStates::GO_TO_LEFT_LANE), states.end());
		break;
	case 1:
		break;
	default:
            /* We hould never get there */
		cout<<"Error: Check it out: lane number not correct "<< current_lane <<endl;
	}

	return states;

}
/* Find the minimum cost trajectory allowed by our current state */
TrajectoryCtrlStates Control::update_state(const std::vector<double> &start_s, const std::vector<double> &start_d,
		std::map<int, Vehicle> &predictions)
{
    UTIL u;
	static bool ResetClockNow = true;
	if(ResetClockNow){
		ResetClockNow = false;
		Timer.reset();
	}
    /* Compute what states are reachable given our current lane */
	vector<TrajectoryCtrlStates> states = get_all_states(start_d);
    
    /* Instantiate and Initialize our car class */
	Vehicle  vehicle({start_s[0], start_s[1], start_s[2], start_d[0], start_d[1], start_d[2]});
    
     /* Compute a set of potential trajectories and assess their risk uning cost functions */
	TrajectoryCtrlCostData data = compute_TrajectoryCtrl_cost_data(vehicle, predictions);
	
    
    double min_cost = INFINITY;
    
    /*Set as default state "STAY IN LANE" */
	TrajectoryCtrlStates min_cost_state = TrajectoryCtrlStates::STAY_IN_LANE;
    
    
    /*For each reachable states compute the cost of trajectories and select the one with minimum  cost */
	for(auto &state: states){
		double cur_cost = calculate_cost(vehicle, state, data);
		if(cur_cost < min_cost){
			min_cost = cur_cost;
			min_cost_state = state;
		}
	}
	
	if(tc_last_state != min_cost_state)
    {
		tc_last_state = min_cost_state;
		previous_lane_used = get_target_lane_id(vehicle, tc_last_state);
		Timer.reset();
		cout<<"SUGGESTED STATE: GOTO LANE = "<<u.which_lane(start_d[0])<<" "<< min_cost_state<<endl;
	}
    /* return the reachable state with minimum cost */
	return min_cost_state;


}

/* Compute the cost of each trajectory */
double Control::calculate_cost(const Vehicle & vehicle, TrajectoryCtrlStates state, TrajectoryCtrlCostData &data)
{
	double cost = 0;
	for (auto& measured : c_cost)
    {
		auto cost_of_this = measured.second;
		double cur_cost = cost_of_this.weight * cost_of_this.cost_func(vehicle, state, data);
		cost += cur_cost;
	}

	return cost;
}
/* Find out where the other cars are located in particular in which lanes they speeding */
TrajectoryCtrlCostData Control::compute_TrajectoryCtrl_cost_data(const Vehicle & vehicle, std::map<int, Vehicle> &predictions)
{
    UTIL u;
	std::map<int, Vehicle> cars_in_front_of_us;
    std::map<int, Vehicle> cars_behind_us;
    

	double ourcar_cur_s = vehicle.start_state[0];//our current location
    
	for (const auto& measured : predictions)
    {
		const Vehicle &othercar = measured.second;
        double othercar_cur_s = othercar.start_state[0];
		if(othercar_cur_s < ourcar_cur_s) //if the car is behind us
        {
            int othercar_lane_id = u.which_lane(othercar.start_state[3]);//lane of this other car behind us
            if(othercar_lane_id < 0 || othercar_lane_id > 2)
            {
                cout<<"Error: unexpected lane in sensor fusion data" <<endl;
                continue;
            }
            if(cars_behind_us.find(othercar_lane_id) == cars_behind_us.end())
            {
                cars_behind_us[othercar_lane_id] = othercar;
                continue;
            }
            if(othercar_cur_s > cars_behind_us[othercar_lane_id].start_state[0])
            {
                cars_behind_us[othercar_lane_id] = othercar;
                cout << "Another car behind us lane: " << othercar_lane_id << endl;
                
            }
          
        }
          else //if the car is in front of us
        {
            
		   int lane_id = u.which_lane(othercar.start_state[3]);
		   if(lane_id <0 || lane_id >2)
           {
			cout<<"Error: unexpected lane in sensor fusion data" <<endl;
			continue;
		   }
		   if(cars_in_front_of_us.find(lane_id) == cars_in_front_of_us.end())//if the car is in our lane
           {
			cars_in_front_of_us[lane_id] = othercar;
			continue;
		   }
		   if(othercar.start_state[0] < cars_in_front_of_us[lane_id].start_state[0])
           {
			cars_in_front_of_us[lane_id] = othercar;
		   }
            
        }
	}

	TrajectoryCtrlCostData data(vehicle, predictions, cars_in_front_of_us, tc_last_state, Timer.elapsed(), previous_lane_used);
	return data;
}

