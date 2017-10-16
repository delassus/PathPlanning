//
//  ChangingLaneRiskEvaluation.hpp
//  Path_Planning
//
//  Created by Hubert de Lassus on 8/12/17.
//
//
#include <vector>
#include "utils.h"
#include "TrajectoryCtrlCost.h"
#include "TrajectoryController.h"

#ifndef ChangingLaneRiskEvaluation_hpp
#define ChangingLaneRiskEvaluation_hpp


class ChangingLaneRiskEvaluation {
public:
    ChangingLaneRiskEvaluation(){};
    ~ChangingLaneRiskEvaluation(){};
    vector<bool> EvaluateRisk(Vehicle myCar, const std::vector<std::vector<double>> &sensor_fusion);
    bool check_if_adjacent_lane(int target_lane,int car_behind_lane);

};

#endif /* ChangingLaneRiskEvaluation_hpp */
