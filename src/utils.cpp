/*
 * utils.cpp
 *
 */

#include "utils.h"
#include <math.h>
#include "constants.h"
#include "TrajectoryCost.h"
#include <sys/time.h>
#include <iostream>
#include <fstream>

//This file contains utility routines grouped under class UTIL

/* Constructor */
UTIL::UTIL()
{
}
const std::string UTIL::currentDateTime()
{
    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;
    
    char buffer [80];
    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));
    
    char currentTime[84] = "";
    sprintf(currentTime, "%s:%d", buffer, milli);
    
    return currentTime;
}
int	UTIL::which_lane(double d)
{
    return int(d / LANE_WIDTH_METERS);
}

/* Destructor */
UTIL::~UTIL() {
	
}


double UTIL::get_lane_dist(int lane_id)
{
	return lane_id*LANE_WIDTH_METERS + LANE_WIDTH_METERS/2.0;
}


double UTIL::logistic(double x)
{
	return 2.0 / (1 + exp(-x)) - 1.0;
}


