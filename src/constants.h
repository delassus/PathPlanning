/*
 * constants.h
 *
 */

#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

const float MAX_DISTANCE_BETWEEN_POINTS = 0.4195;
const float MIN_SPEED = 0;
const float MAX_D = 11.50;
const float MIN_D = 0.5;
const float LANE_WIDTH_METERS = 4;
const int RECYCLED_POINTS = 10;
const double PERIOD = 0.02;
const float SIGMA_S[] = {10.0, 4.0, 2.0}; // s, s_dot, s_double_dot
const float SIGMA_D[] = {1.0, 1.0, 1.0};
const float SIGMA_T = 2.0;
const float MAXIMUM_JERK_ALLOWED = 9;// # m/s/s/s
const float MAX_ACCEL= 9;// # m/s/s
const float EXPECTED_JERK_PER_SECOND = 2;// # m/s/s
const float MAX_METERS_PER_SECOND_SPEED = 21;
const float SAFE_DISTANCE_BUFFER = 3*MAX_METERS_PER_SECOND_SPEED;
const float EXPECTED_ACC_PER_SECOND = 1;// # m/s
const float VEHICLE_RADIUS = 1.5;
const float COLLISION_DISTANCE = 3;
const int NUMBER_OF_POINTS = 150;
const double MAX_S = 6945.554;
const double FRONT_SECURITY_BUFFER = 55.0;
const double BACK_SECURITY_BUFFER = 50;//30;
const double HYSTERESIS_COST = 12000;
const double REAR_END_SECURITY_DISTANCE = 50.0;
const double FRONT_SECURITY_DISTANCE = 55;//25.0;
const double FIFTY_MILES_PER_HOURS = 22.3; //expressed in meters per seconds
#endif /* _CONSTANTS_H_ */
