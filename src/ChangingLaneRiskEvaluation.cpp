//
//  ChangingLaneRiskEvaluation.cpp
//  Path_Planning
//
//  Created by Hubert de Lassus on 8/12/17.
//
//

#include "ChangingLaneRiskEvaluation.h"
#include "cmath"
#include "constants.h"
#include <stdio.h>
#include <iostream>
#include "utils.h"

using namespace std;

bool ChangingLaneRiskEvaluation::check_if_adjacent_lane(int my_car_lane,int car_behind_lane)
{
    
    if(car_behind_lane == my_car_lane)
    {
        return false;
    }
    else if((car_behind_lane == 0) && (my_car_lane == 1))
    {
        return true;
    }else if((car_behind_lane == 0) && (my_car_lane == 2))
    {
        return false;
    }
    else if((car_behind_lane == 1) && (my_car_lane == 0))
    {
        return true;
    }else if((car_behind_lane == 1) && (my_car_lane == 2))
    {
        return true;
    }else if((car_behind_lane == 2) && (my_car_lane == 0))
    {
        return false;
    }else if((car_behind_lane == 2) && (my_car_lane == 1))
    {
        return true;
    }else{
        return true;
    }
}


vector<bool> ChangingLaneRiskEvaluation::EvaluateRisk(Vehicle myCar, const std::vector<std::vector<double>> &sensor_fusion)
{
    /* In this function we list by lane the cars that are behind us at a distance that is a potential risk (40m) */
    /* We keep only the closest car                                                                              */
    /* We measure for each of these cars its speed relative to us                                                */
    /* We evaluate its trajectory for the next ten seconds                                                       */
    /* if this trajectory s(t) reaches our s(t) then there is a collision                                        */
    /* If there is no collision risk to move to that lane we set the risk to zero otherwise we set it to 1       */
    
    
    double Mycar_s = myCar.start_state[2];
    double Mycar_d = myCar.start_state[3];
    

    double Mycar_s_dot = sqrt(myCar.vx*myCar.vx + myCar.vy*myCar.vy);
    vector<double> myCarState3seconds = myCar.state_in(3.0);
    

    vector<bool> ForbiddenLane(3, true);/* in this vector we will indicate which lanes are safe to move to and which are not */
    UTIL u;
    
    /* for each car in the loop compute its distance to us its relative speed and its lane */
    for (const auto& othercar : sensor_fusion)
    {
        double othercar_vx = othercar[3];
        double othercar_vy = othercar[4];
        double othercar_s = othercar[5];
        double othercar_d = othercar[6];
        double othercar_s_dot = sqrt(othercar_vx*othercar_vx + othercar_vy*othercar_vy);
        double ds = othercar_s_dot - Mycar_s_dot;
        Vehicle carbehind;
        carbehind.vx = othercar[3];
        carbehind.vy = othercar[4];
        carbehind.s = othercar[5];
        carbehind.d = othercar[6];
        carbehind.s_dot = sqrt(othercar_vx*othercar_vx + othercar_vy*othercar_vy);
    
        /* if "d" in Frenet coordinates does not make sense (outside of the road) let's by pass this algorithm altogether */
        if(othercar_d < 0)
        {
            continue;
        }
        /* Taking care of the loop transition between 6945.554 and 0 issues.
           This transition issue occurs when we start a new loop and cross our starting line.
            Some waypoints in our trajectory indicate more than 6900 others indicate less than 150.
         */
        if(Mycar_s >= 0 && Mycar_s <= 300)
        {
            //if mycar is just starting a lap
            Mycar_s = Mycar_s + MAX_S;
        }
        //if the other car has just completed one lap
        if(othercar_s >= 0  && othercar_s <= 300)
        {
            othercar_s = othercar_s + MAX_S;
        }
        bool adjacent_lane =false;
        
        /* Compute what is our current lane */
        myCar.lane = u.which_lane(myCar.d);
        /* Compute what is the lane of the car behind us */
        carbehind.lane = u.which_lane(carbehind.d);
        /* Mark the adjacent lanes as not accessible for us if an other car is there within short distance */
        adjacent_lane = check_if_adjacent_lane(myCar.lane,carbehind.lane);
        
        
        /* case other car is behind in adjacent lane */
       if((othercar_s <= Mycar_s) && (((Mycar_s - othercar_s) <= REAR_END_SECURITY_DISTANCE) && (Mycar_s_dot <= othercar_s_dot) && (adjacent_lane==true)))
        {
           double current_distance =  sqrt(((othercar_s - Mycar_s) * (othercar_s - Mycar_s) + (othercar_d - Mycar_d) * (othercar_d - Mycar_d)));
           if(current_distance < REAR_END_SECURITY_DISTANCE)
           {
               ForbiddenLane.at(u.which_lane(othercar_d)) = false;
               cout << "WARNING CAR BEHIND COMING FAST ON LANE: " << u.which_lane(othercar_d) << " Difference of speed: " << ds << endl;
               cout << "My lane: "<< myCar.lane << " car behind on lane: "<< carbehind.lane << "  Distance "  << current_distance << endl;
               cout << "My s_dot: "  << Mycar_s_dot << " car behind s_dot: "<< othercar_s_dot << endl;
               
           }
        /* case other car is close in font of us in adjacent lane */
        }else if((othercar_s >= Mycar_s) && (((Mycar_s - othercar_s) <= SAFE_DISTANCE_BUFFER) && (adjacent_lane==true)))
        {
            double current_distance =  sqrt(((othercar_s - Mycar_s) * (othercar_s - Mycar_s) + (othercar_d - Mycar_d) * (othercar_d - Mycar_d)));
            if(current_distance < FRONT_SECURITY_DISTANCE)
            {
                ForbiddenLane.at(u.which_lane(othercar_d)) = false;
                cout << "WARNING CAR AHEAD ON ADJACENT LANE: " << u.which_lane(othercar_d) << " Difference of speed: " << ds << endl;
                cout << "My lane: "<< myCar.lane << " car ahead on lane: "<< carbehind.lane << "  Distance "  << current_distance << endl;
                cout << "My s_dot "  << Mycar_s_dot << " car ahead s_dot: "<< othercar_s_dot << endl;
                
            }
        }
    }
    return ForbiddenLane;/* a value "false" in ForbiddenLane vector means don't go there in the next move , there is a risk of collision */
}

