/*
 * TrajectoryController.cpp
 *
 */

#include "TrajectoryController.h"
#include "spline.h"
#include "TrajectoryCost.h"
#include "cmath"
#include "utils.h"
#include "constants.h"

using namespace std;

static tk::spline tc_x_spline;
static tk::spline tc_y_spline;
static tk::spline tc_dx_spline;
static tk::spline tc_dy_spline;
constexpr double pi() { return M_PI; }


//Listed below are a series of functions taken from the helper file

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; 
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;
    
    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }
    
    int wp2 = (prev_wp+1)%maps_x.size();
    
    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    
    double seg_s = (s-maps_s[prev_wp]);
    
    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
    
    double perp_heading = heading-pi()/2;
    
    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);
    
    return {x,y};
    
}


int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
    
    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];
    
    double heading = atan2( (map_y-y),(map_x-x) );
    
    double theta_pos = fmod(theta + (2*pi()),2*pi());
    double heading_pos = fmod(heading + (2*pi()),2*pi());
    double angle = abs(theta_pos-heading_pos);
    if (angle > pi()) {
        angle = (2*pi()) - angle;
    }
    
    cout << "heading:" << heading << " diff:" << angle << endl;
    
    if(angle > pi()/2)
    {
        closestWaypoint = (closestWaypoint + 1) % maps_x.size();
        
    }
    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
																						{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

/* Class  Destructor */
TrajectoryController::~TrajectoryController()
{
    
}

/* Class Constructor */
TrajectoryController::TrajectoryController(const vector<double> &maps_s, const vector<double> &maps_x, vector<double> &maps_y,
		const vector<double> &maps_dx, const vector<double> &maps_dy)
{
	tc_map_s = maps_s;
	tc_map_x = maps_x;
	tc_map_y = maps_y;
	tc_map_dx = maps_dx;
	tc_map_dy = maps_dy;

	tc_x_spline.set_points(tc_map_s, tc_map_x);
	tc_y_spline.set_points(tc_map_s, tc_map_y);
	tc_dx_spline.set_points(tc_map_s, tc_map_dx);
	tc_dy_spline.set_points(tc_map_s, tc_map_dy);
	previous_path_points = 0;
	tc_current_state = TrajectoryCtrlStates::STAY_IN_LANE;

}

//This function compute for each car within sensor_fusion vector of vehicle its start state
std::map<int, Vehicle> TrajectoryController::get_predictions(const std::vector<std::vector<double>> &sensor_fusion,
                                                             double start_s,const std::vector<double> &car_state){
    int predicted_vehicle = 0;
    double closest_car_id = -1;
    double closest_distance = INFINITY;
    double car_s = car_state[2];
    double car_d = car_state[3];
    std::map<int, Vehicle> predictions;
    
    for (const auto& othercar : sensor_fusion)
    {
        int othercar_id = othercar[0];
        double vx = othercar[3];
        double vy = othercar[4];
        double s = othercar[5];
        double d = othercar[6];
        double s_dot = sqrt(vx*vx + vy*vy);
        if(d < 0)
        {
            continue;
        }
        
        if(start_s <= MAX_S && start_s >= MAX_S - 300){
            
            if(s >= 0  && s <= 300)
            {
                s = s + MAX_S;
            }
        }
        
        
        double new_s = s + s_dot * RECYCLED_POINTS * PERIOD;
        
        double current_distance =  sqrt(((s - car_s) * (s - car_s) + (d - car_d) * (d - car_d)));
        if(current_distance < closest_distance)
        {
            closest_distance = current_distance;
            closest_car_id = predicted_vehicle;
        }
        std::vector<double> start_state = {new_s, s_dot, 0, d, 0, 0};
        
        predictions[othercar_id] = Vehicle(start_state);
    }
    
    return predictions;
}



//This function assess the risk we take by changing lane. After risk assessment, the function decides if we will change lane or not.
// To do this it just checks the no_go_lanes vector to see if our target lane is open or close for us.
// The target lane will be close for us if there is already a vehicle in that lane in a position that could lead to a collision.

TrajectoryCtrlStates TrajectoryController::checkRisk(int our_current_lane,int target_lane,vector<bool> no_go_lanes)
{
    
    if((  no_go_lanes.at(target_lane) == false) && (target_lane != our_current_lane))
    {
        cout << "WARNING: vehicle arriving behind! cannot go to lane: "<< target_lane << " from lane: "<< our_current_lane   << endl;
        return TrajectoryCtrlStates::STAY_IN_LANE;
    }
    else if((target_lane == 0) && (our_current_lane == 1))
    {
        cout << "COMMAND: No vehicle arriving behind! go to lane: "<< target_lane << " from lane: "<< our_current_lane   << endl;
        
          return TrajectoryCtrlStates::GO_TO_LEFT_LANE;
    }else if((target_lane == 1) &&(our_current_lane == 0))
    {
        cout << "COMMAND: No vehicle arriving behind! go to lane: "<< target_lane << " from lane: "<< our_current_lane   << endl;
        
          return TrajectoryCtrlStates::GO_TO_RIGHT_LANE;
    }else if((target_lane == 1) &&(our_current_lane == 2))
    {
        cout << "COMMAND: No vehicle arriving behind! go to lane: "<< target_lane << " from lane: "<< our_current_lane   << endl;
        
            return TrajectoryCtrlStates::GO_TO_LEFT_LANE;
    }
    else if((target_lane == 2) &&(our_current_lane == 1))
    {
        cout << "COMMAND: No vehicle arriving behind! go to lane: "<< target_lane << " from lane: "<< our_current_lane   << endl;
        
            return TrajectoryCtrlStates::GO_TO_RIGHT_LANE;
    }else{
        cout << "COMMAND: STAY IN LANE "<<  our_current_lane   << endl;
        
            return TrajectoryCtrlStates::STAY_IN_LANE;
    }
}


//This function compute what could be our target lane depending upon our current lane and the path planner suggested lane change
int TrajectoryController::get_our_target_lane(TrajectoryCtrlStates suggested_state,int our_current_lane)
{
    //Lanes our ordered like this left lane:0, middle lane:1, righ lane:2
  int target_lane = our_current_lane;
   if((our_current_lane == 0) && (suggested_state == TrajectoryCtrlStates::GO_TO_LEFT_LANE))
   {
       target_lane = our_current_lane;
   }
   else if((our_current_lane == 0) && (suggested_state == TrajectoryCtrlStates::GO_TO_RIGHT_LANE))
   {
       target_lane = 1;
   }
   else if((our_current_lane == 0) && (suggested_state == TrajectoryCtrlStates::STAY_IN_LANE))
   {
       target_lane = our_current_lane;
   }
   else if((our_current_lane == 1) && (suggested_state == TrajectoryCtrlStates::GO_TO_LEFT_LANE))
   {
       target_lane = 0;
   }
   else if((our_current_lane == 1) && (suggested_state == TrajectoryCtrlStates::GO_TO_RIGHT_LANE))
   {
       target_lane = 2;
   }
   else if((our_current_lane == 1) && (suggested_state == TrajectoryCtrlStates::STAY_IN_LANE))
   {
       target_lane = our_current_lane;
   }
   else if((our_current_lane == 2) && (suggested_state == TrajectoryCtrlStates::GO_TO_LEFT_LANE))
   {
       target_lane = 1;
   }
   else if((our_current_lane == 2) && (suggested_state == TrajectoryCtrlStates::GO_TO_RIGHT_LANE))
   {
       target_lane = our_current_lane;
   }
   else if((our_current_lane == 2) && (suggested_state == TrajectoryCtrlStates::STAY_IN_LANE))
   {
       target_lane = our_current_lane;
   }
    return  target_lane;
}


// This function computes the next way points that will be forwarded to the simulator
//(1) We first collect our current state
//(2) We compute the current state of all vehicles within range
//(3) Based on (2) and (3) the path planner computes a suggested future state for us
//(4) Depending upon the possible closure of our target lane in the suggested state we decide for an imperative future state.
//(5) The imperative future state will be the command that generates our next_way_points.
// Note that the state becomes "imperative", meaning that there is no coming back once initiated. This requirement is needed
// to avoid excessive time passed in between two lanes. If the vehicle could change its mind in the middle of a lane change,
// then the time to roll back to the initial lane would be excessive. The car would be considered lingering "outside of lane"
// by the simulator and a traffic violation would be flagged.


void TrajectoryController::generate_next_waypoints(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
                                                   const std::vector<std::vector<double>> &sensor_fusion, std::vector<bool> no_go_lanes)
{
    UTIL u;
    static TrajectoryCtrlStates previous_state = TrajectoryCtrlStates::STAY_IN_LANE;
    TrajectoryCtrlStates imperative_state;
	std::vector<std::vector<double>> start_state = get_start_state(car_state, previous_path_x,previous_path_y, end_path_s, end_path_d);
    
	std::vector<double> &start_s = start_state[0];
	std::vector<double> &start_d = start_state[1];
	std::map<int, Vehicle> predictions = get_predictions(sensor_fusion, start_s[0], car_state);
	double T = 5;
    
    //Ask the controller for our next state. The controller suggests a next state
	TrajectoryCtrlStates suggested_state = tc_Control.update_state(start_s, start_d, predictions);


    vector<double> dv = start_state[1];
    
    
    int our_current_lane = u.which_lane(dv.at(0));
    
    //Find out what is the target lane of the suggested state
    int target_lane = get_our_target_lane(suggested_state,our_current_lane);
    
    // Check if a change lane was initiated earlier and has not been completed yet
    // Once we have started a lane change  we need to complete it rather than initiate a new one
    
    if((previous_state == suggested_state)&&(previous_state != TrajectoryCtrlStates::STAY_IN_LANE))
    {
        imperative_state = suggested_state;
        previous_state = suggested_state;
    }else{
        if((start_s.at(0) > (MAX_S - 300))||(start_s.at(0) < 300 ))
        {
            imperative_state = TrajectoryCtrlStates::STAY_IN_LANE;
        }else{
        /* Once a lane change is completed we can assess our next move */
            imperative_state = checkRisk(our_current_lane,target_lane,no_go_lanes);
        }
        previous_state = imperative_state;
    }
    
    
    TrajectoryModel chosenTrajectory;
    
    
    //Depending upon the chosen next state, generate a command
    if (imperative_state == TrajectoryCtrlStates::GO_TO_LEFT_LANE)
    {
        /* what is the lane we want to go to ? */
        cout << "COMMAND: GO TO LEFT LANE " << endl;
        //Lane Change to the Left
		chosenTrajectory = tc_trajectory.LANE_CHANGE(start_s, start_d,T, predictions, true);
	}
	else if (imperative_state == TrajectoryCtrlStates::GO_TO_RIGHT_LANE){
        //Lane Change to the Right
        cout << "COMMAND: GO TO RIGHT LANE " << endl;
		chosenTrajectory = tc_trajectory.LANE_CHANGE(start_s, start_d,T, predictions, false);
	}
	else /*(imperative_state == TrajectoryCtrlStates::STAY_IN_LANE)*/{
        //Stay in Lane
        cout << "COMMAND: STAY IN LANE " << endl;
		chosenTrajectory = tc_trajectory.keep_lane(start_s, start_d, T, predictions);
	}
    //Compute the next way points that fit our new trajectory
	adjust_next_waypoints(chosenTrajectory);
    
}


//This function computes the start state of the vehicle
std::vector<std::vector<double>> TrajectoryController::get_start_state(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d){

	double car_s = car_state[2];
	double car_d = car_state[3];
	double car_speed = car_state[5];
	
	car_speed = (car_speed * 1609.34)/3600;//in meter per seconds

	tc_trajectory.tr_current_car_speed = car_speed;

	
	vector<double> start_s = {car_s,car_speed,0};
	vector<double> start_d = {car_d,0,0};

	if(previous_path_x.size() != 0){
		
		std::vector<std::vector<double>> start_state = recycle_previous_path(previous_path_x, previous_path_y, end_path_s, end_path_d);
		start_s = start_state[0];
		start_d = start_state[1];
	}

	return {start_s,start_d};
}



void TrajectoryController::adjust_next_waypoints(const TrajectoryModel &chosenTrajectory)
{
    
    // This function regulates our car next way points to please the simulator
    //Two types of adjustments are conducted
    //(1) Adjustment of "d" parameter
    //    The simulator complains spuriously of "out of lane" position when our vehicle is on the right lane.
    //    This is possibly a fake signal. in order to silence the simulator, when our car is on the right lane,
    //    its position is slighly tweaked to the left instead of being right in the middle of the lane.
    //    This way the simulator is pleased and no out of lane signal is flagged.
    //
    //(2) The speed of our car is capped when its comes close to the speed limit of 50 miles per hour.
    //    Since adjusting the goal s of the trajectory is not precise enough to prevent spurious excessive speed signals from the simulator,
    //    a more drastic aproach is being implemented here.
    //    What we do here is we compute the speed between every single points in the trajectory.
    //    If one single point position generates an excessive speed relative to the previous point, this transgressing point is adjusted
    //    by the right "s" amount that will keep it within the speed limit. All way points after that first transgressing point are similarly adjusted.
    //    All way points before the transgressing way point are left untouched.
    //    The bottom line is that no way point can be further than 0.4195 (in s coordinates) from its previous way point.
    
    // if rule (1) and (2) are enforced there should be be no "out of lane " signal and not "speed violation" signals from the simulator.
    

	tc_next_x_vals = {};
	tc_next_y_vals = {};
    double distance_between_points = 0.0;
    double automatic_speed_reduction = 0.0;
    static double max_dist_b_p = 0.0;
    double next_point_s = 0.0;
    
	double t = chosenTrajectory.t;
	std::vector<double> s_coeff = chosenTrajectory.s_coeff;
	vector<double> s_dot_coeff = differentiate(s_coeff);
	vector<double> s_dot_dot_coeff = differentiate(s_dot_coeff);

	std::vector<double> d_coeff = chosenTrajectory.d_coeff;
	vector<double> d_dot_coeff = differentiate(d_coeff);
	vector<double> d_dot_dot_coeff = differentiate(d_dot_coeff);

	double dt = PERIOD;
	double cur_t = 0;

	int count = NUMBER_OF_POINTS - tc_last_waypoints_s.size();
	for(int k = 0; k < count; k++)
    {
		if(cur_t > t)
        {
			break;
		}
		double cur_s = to_equation(s_coeff, cur_t);
		if(cur_s > MAX_S)
        {
			cur_s = cur_s - MAX_S;
		}
		vector<double> s = {cur_s,to_equation(s_dot_coeff, cur_t),to_equation(s_dot_dot_coeff, cur_t)};
		tc_last_waypoints_s.push_back(s);

		vector<double> d = {to_equation(d_coeff, cur_t),to_equation(d_dot_coeff, cur_t),to_equation(d_dot_dot_coeff, cur_t)};
		tc_last_waypoints_d.push_back(d);

		cur_t += dt;
	}
	for(int k = 0; k < tc_last_waypoints_s.size(); k++)
    {
		double cur_s = tc_last_waypoints_s[k][0];
		double cur_d = tc_last_waypoints_d[k][0];
        
        /* Check that for each way point the speed limit is not exceeded */
        if( k < tc_last_waypoints_s.size()-1)
        {
             next_point_s = tc_last_waypoints_s[k+1][0];
            if(next_point_s > cur_s)
            {
               distance_between_points = next_point_s - cur_s;
                if(distance_between_points > max_dist_b_p)
                {
                    max_dist_b_p = distance_between_points;
                }
            }else{
                /* check possible bug here */
                distance_between_points = (MAX_S - cur_s) + next_point_s;
                if(distance_between_points > max_dist_b_p)
                {
                    max_dist_b_p = distance_between_points;
                    
                }
               
            }
            if(distance_between_points > MAX_DISTANCE_BETWEEN_POINTS)
            {
                if((k%10)==0)
                {
                   cout << "SPEED CAPPED (MAX is: " << MAX_DISTANCE_BETWEEN_POINTS <<")  :" << distance_between_points << endl;
                }
                automatic_speed_reduction = distance_between_points - MAX_DISTANCE_BETWEEN_POINTS;
                cur_s = cur_s - automatic_speed_reduction;
                for (int j = k; j < tc_last_waypoints_s.size();j++)
                {
                    tc_last_waypoints_s[j][0] = tc_last_waypoints_s[j][0] - automatic_speed_reduction;
                }
            }
        }
        
        if(cur_d > 9.8) //0-2-4  4-6-8 8-10-12
        {
            cur_d = 9.8;
            
        }else if(cur_d < 2.0)
        {
               cur_d = 2.0;
        }
		vector<double> xy = convert_sd_to_xy(cur_s, cur_d);
		tc_next_x_vals.push_back(xy[0]);
		tc_next_y_vals.push_back(xy[1]);
        
	}
	previous_path_points = tc_next_x_vals.size();
    cout << "Max distance between point: " << max_dist_b_p << endl;//0.41
}


std::vector<std::vector<double>> TrajectoryController::recycle_previous_path(const std::vector<double> &previous_path_x,
                                                                             const std::vector<double> &previous_path_y, double end_path_s,double end_path_d)
{

    // This function checks which points the simulator has consumed and how we can stitch together the previous path with the next path in a smooth way
    
    
    
    vector<double> start_s = {};
    vector<double> start_d = {};
    
    int points_traveled =  previous_path_points - previous_path_x.size();
    start_s = tc_last_waypoints_s[points_traveled + RECYCLED_POINTS];
    start_d = tc_last_waypoints_d[points_traveled + RECYCLED_POINTS];
    
    vector<vector<double>> track_s;
    vector<vector<double>> track_d;
    for(int k = 0; k < previous_path_points; k++)
    {
        if(k   < points_traveled)
        {
            continue;
        }
        if( k >= points_traveled + RECYCLED_POINTS)
        {
            break;
        }
        track_s.push_back(tc_last_waypoints_s[k]);
        track_d.push_back(tc_last_waypoints_d[k]);
    }
    
    tc_last_waypoints_s = track_s;
    tc_last_waypoints_d = track_d;
    
    return {start_s,start_d};
    
}

//Class Wrapper for getFrenet()
vector<double> TrajectoryController::getFrenet_Coordinates(double x, double y, double theta)
{
    return getFrenet( x,  y,  theta, tc_map_x, tc_map_y);
}
//Class wrapper for getXY()
vector<double> TrajectoryController::getXY_Coordinates(double s, double d)
{
	return getXY(s, d, tc_map_s, tc_map_x, tc_map_y);
}


// convert s and d to x and y coordinates using cubic spline interpolation
vector<double> TrajectoryController::convert_sd_to_xy(const double s, const double d)
{
	const double x_edge = tc_x_spline(s);
	const double y_edge = tc_y_spline(s);
	const double dx = tc_dx_spline(s);
	const double dy = tc_dy_spline(s);

	const double x = x_edge + dx * d;
	const double y = y_edge + dy * d;

	return {x, y};
}




bool TrajectoryController::lane_change_manager(const std::vector<double> &car_state, const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d,
		const std::vector<std::vector<double>> &sensor_fusion)
{
    // This function remove by passed way points no more needed by the simulator and prepare valid way points for the simulator
    // for the case when we change lane
    
	if(tc_current_state == TrajectoryCtrlStates::STAY_IN_LANE)
    {
		return false;
	}

	if(previous_path_x.size() < 50)
    {
		return false;
	}
	int points_traveled =  previous_path_points - previous_path_x.size();

	//remove bypassed waypoints
	vector<vector<double>> track_s;
	vector<vector<double>> track_d;
	for(int k =0; k < previous_path_points; k++)
    {
		if(k < points_traveled)
        {
			continue;
		}
		track_s.push_back(tc_last_waypoints_s[k]);
		track_d.push_back(tc_last_waypoints_d[k]);
	}


	tc_last_waypoints_s = track_s;
	tc_last_waypoints_d = track_d;
	previous_path_points = tc_last_waypoints_s.size();
    
	tc_next_x_vals = {};
	tc_next_y_vals = {};
    
	for(int k = 0; k < tc_last_waypoints_s.size(); k++)
    {
		double cur_s = tc_last_waypoints_s[k][0];
		double cur_d = tc_last_waypoints_d[k][0];
    
		vector<double> xy = convert_sd_to_xy(cur_s, cur_d);
		tc_next_x_vals.push_back(xy[0]);
		tc_next_y_vals.push_back(xy[1]);
    
	}
	
	return true;
}
