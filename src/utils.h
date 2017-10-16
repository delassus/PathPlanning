/*
 * utils.h
 *
 */

#ifndef _UTIL_H_
#define _UTIL_H_


#include <iterator>
#include <chrono>

#include <vector>
#include <map>
using namespace std;


class TaskTimer
{
public:
	
	void reset() { start_time_ = clock_::now(); }
	double elapsed() const {
		return std::chrono::duration_cast<chrono::milliseconds>
		(clock_::now() - start_time_).count(); }
    TaskTimer() : start_time_(clock_::now()) {}
private:
	typedef std::chrono::high_resolution_clock clock_;

	std::chrono::time_point<clock_> start_time_;
};

class Vehicle{


public:
    
	std::vector<double> start_state;
    double distance_to_our_car;
    double lane;
    bool behind_us;
    bool in_font_of_us;
    double relative_speed;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double s_dot;
    double yaw;
    double speed;
    
	Vehicle(const std::vector<double> &start){
		start_state = start;
	}
	Vehicle(){

	}
    ~Vehicle(){}
	
	std::vector<double> state_in(double t) const{
		std::vector<double> s = {start_state[0],start_state[1],start_state[2]};
		std::vector<double> d = {start_state[3],start_state[4],start_state[5]};
		std::vector<double> state = {
				s[0] + (s[1] * t) + s[2] * t* t / 2.0,
				s[1] + s[2] * t,
				s[2],
				d[0] + (d[1] * t) + d[2] * t* t / 2.0,
				d[1] + d[2] * t,
				d[2],
		};
		return state;
	}
};


class TrajectoryModel{
public:
	TrajectoryModel(const std::vector<double> &s_goal_p, const std::vector<double> &d_goal_p,double t_p,
			const std::vector<double> &plain_s_p,const std::vector<double> &plain_d_p,
			double plain_trajectory_p, double buffer_zone_p){
		s_goal = s_goal_p;
		d_goal = d_goal_p;
		t = t_p;
		plain_s = plain_s_p;
		plain_d = plain_d_p;
		plain_trajectory = plain_trajectory_p;
		TooRisky = false;
		buffer_zone = buffer_zone_p;

	}
	TrajectoryModel(){

	}
    double t;
    double plain_trajectory;
    bool TooRisky;

    std::vector<double> plain_s;
    std::vector<double> plain_d;
    double buffer_zone;

	std::vector<double> s_goal;
	std::vector<double> d_goal;
	std::vector<double> s_coeff;
	std::vector<double> d_coeff;
};





class UTIL {
public:
	UTIL();
	//std::map<int, Vehicle> monitor_traffic(std::vector<std::vector<double>> sensor_fusion);
	virtual ~UTIL();
    int	which_lane(double d);
    double get_lane_dist(int lane_id);
    const std::string currentDateTime();
    double logistic(double x);
};


#endif /* _UTIL_H_ */
