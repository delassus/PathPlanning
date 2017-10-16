# **Goal**
In this project, the goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving 
traffic all by using localization, sensor fusion, and map data. The project runs inside a Trinity simulator where a highway with heavy traffic and six lanes is simulated. The highway is a 6.75 miles loop where the speed limit is 50 miles per hour.  The simulator allows manual driving with the keyboard or a joystick by selecting manual mode on the operational screen where the car is driving. For this project however, we are interested by autonomous driving so we will deselect manual mode on the screen and the car will behave autonomously.
The path planner algorithm will be steering the car. Receiving information from sensors such as LIDAR, Radars, Video Cameras the path planner keeps track of the surrounding traffic and compute the car trajectory for the next 0.2 seconds and possible follow up points for the next 7 seconds.  The trajectory has to stitch smoothly with the previous points in the path to avoid stop and go as well as jerks. 

# **Bosch Udacity Challenge Path Planning winner**
This code won the "Bosch-Udacity Path Planning Challenge" in September 20017. The Challenge involved steering an autonomous car on a simulated highway with heavy traffic for 10 miles. Fastest Time, passenger comfort (smooth trajectories and accelerations)  and safety were the criteria to win. Note that the Bosch Udacity simulator highway used for the Challenge differed from the one used here.

# **Design criteria**
- The car must not go over the speed limit of 50mph
- The car must not exceed a total acceleration of 10 m/s^2 and a jerk (lateral acceleration) of 10 m/s^3.
- The car must not come into contact with any of the other cars on the road.
- The car must not spend more than a 3 second time length in between lanes when changing lanes and at every other time the car must stay inside one of the 3 lanes on the right hand side of the road.
- The car must be able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

# **Usage**
When running the simulator, the car that drives autonomously is the black car in front of us.
By clicking on the screen and moving around with the mouse or finger, it is possible to change the angle of the camera that looks at the car. So we can view the car from the rear, from the top or from the front or lateral views. Since the highway is a loop, this feature comes handy to keep a full view of the car from behind and slightly up. This is the best camera position to check what the car is doing.
Again, make sure the Manual Mode of the simulator is not selected.

# **To run this code** 
* (1) Download the simulator: git clone https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2.git
* (2) Down load this code:  git clone https://github.com/delassus/PathPlanning.git
* (2) create a build directory: mkdir build, cd build and run Cmake ..  under build directory
* (3) compile the code using Xcode or make -j8
* (4) Run the simulator, select a graphics definition, then select play!, select  project 1: path planning. Make sure that Manual mode is not selected on the screen where the car is running.

* (5) Run the code, the car will move in the simulator. Watch how the car follows the car in front keeping a safe distance of 50 meters and how it changes lane when possible if the front car is slow. Check how the car slows if the front car slows and how it accelerate to 49 miles an hour when possible.

# **Algorithm**
The green line in front of the car is the path planned by the algorithm. This green line is made up of green dots at 0.2 seconds intervals.They show where the car should go in the next 0.2 seconds These dots are continuously computed by the path planning algorithm and sent to the simulator. If the distance between the dots increases, the car is accelerating. The trajectory is created by using splines between a start point (where the car is now) and where the car has to be in the immediate future. A bunch of candidate trajectories is generated between the start point and the end point. Then, a series of 14 cost functions are computed for each trajectory. The 14 costs are added to constitute the total cost of a given trajectory. The path planner eventually will choose the trajectory with the least cost.

# **Key points that the path planner must meet**
* (1) Send to the simulator every 0.2 seconds a vector of way points that stitches smoothly to the previous vector of way points so that the trajectory remains smooth.
* (2) Keep a close look at the rapidly evolving traffic around the car to avoid collisions while driving at the optimum speed. Keeping a balance between safety and performance.
* (3) The path planner must continuously assess the trajectory to avoid jerks and excessive speed, acceleration or deceleration.
* (4) The path planner must find out if and when a change of lane is in order and safe.
If these points are not met at any time, the simulator will detect it and a pop up window will flag the violation. In particular excessive speed, excessive acceleration, excessive jerk (lateral acceleration) and excessive time between lanes will be flagged as incident and wil restart the "No incident" timer.

# **Issues and Solutions**
* (1) The simulator was often flagging the car as "out of lane" when driving on the right lane. It appeared that this was probably an erroneous signal. However to please the simulator and silence this signal a bias of 20 centimeters was introduced for the "d" Frenet parameter when the car was driving on the right lane. 
* (2) The simulator was often flagging the car with "exceed the speed limit" signal even though the car seemed to be within the speed limit. To address this issue, the distance between each way point was capped such that if the path plan was executed at a frequency of one point every 0.2 seconds the car speed would be under 48 miles per hour. This precaution did not completely solve the problem but dramatically decreased the occurrences of speed limit violation. To completely eliminate the speed limit violation a Real Time Operating system would be needed to guarantee the frequency of the path updates. The solution to set the speed much lower than the 50 mph limit was found to penalize the performance too much and was not implemented. A frequency of speed violation of one violation per hour was accepted (That is less than one tenth of a second violation).
* (3) Since the simulator is a loop of 6.9 miles, this creates a problem when the car completes a loop: the Frenet parameter "d" jumps from 6945 to 0 in one instant.
This issue is addressed by handling separately the transition zone 300 meters before and 300 meters after the 6945 to zero transition line. However we notice that on the left lane (lane zero) the simulator often brings the car almost to a stop when entering in the transition zone. This does not happen in the two other lanes but only on the left lane. No solution was found to fix this issue.
* (4) To avoid collisions is one of the most important task of the path planner. This requirement is only partially met unfortunately. The main reason being the erratic sometimes frantic behavior of cars on the road. On many occasions car simply crash into you from the side or from the rear as if the driver was suicidal. Some other times two or more cars collide and loosing control at high speed eventually crash into you. These situations cannot be handled by the path planner without breaching the speed limit and the jerk/acceleration requirements.
The path planner in this implementation uses two approaches simultaneously to avoid collisions. First a collision risk is evaluated with a cost function by the trajectory controller. Second a continuous monitoring of the positions of other car around us is conducted. Lanes that contain cars immediately behind or immediately in front of us are simply put on a no "change lane list". No trajectory ending on such lanes are computed until these lanes are free of traffic.  

# **How the trajectory is computed**
First a start state and an end state are computed using Frenet coordinates s and d. The start state is where the car is and the end state is 80 meters in front of us.
Using these two bounding conditions a bunch of trajectories are computed with random perturbations.
Then a series of cost functions evaluate these trajectories for safety performance and passenger comfort. A weight is added for each cost function. This weight is then adjusted by trial and error for each function until a right combination is found.
# 14 Cost functions
* time_diff: penalizes differences between the expected time to goal and the realized time.
* s_diff: penalizes difference between expected s and realized s.
* d_diff: penalizes difference between expected d and realized d;
* collision: penalizes trajectories coming closer than 1.5 meter to another vehicle. 
* buffer: penalizes trajectories that tailgate the leading vehicle.
* stays_on_road: penalizes trajectories that lead the vehicle outside of the road.
* exceeds_speed_limit: penalizes trajectories that exceed the speed limit.
* total_accel: penalizes too much acceleration
* max_accel: penalizes excessive instant acceleration.
* total_jerk_cost: penalizes excessive average jerk
* max_jerk: penalizes excessive instant jerk.
* speed cost : penalizes choosing a lane with a slow leading vehicle.
* lane change cost: penalizes changing lane too often
* lane change hysteresis: penalizes toggling between lanes.

# **State Machine**
* The addition of these cost functions lead to a "suggested state" by the path planner. This suggestion is accepted by the state machine if it leads to a lane with no cars in the vicinity. Otherwise an "imperative state" is superseding the "suggested state". The imperative state can be "continue the change lane procedure that was started earlier", or just stay in lane.


* At this stage the car drives around the circuit consistently. The trajectory is smooth even if a low frequency oscillation occurs especially on straight road.

* The path planner consists of a state machine with three states: STAY IN LANE, GO RIGHT, and GO LEFT. Once the decision is made to leave the STAY IN LANE state, the command to change lane cannot be rolled back. The car follows the trajectory to change lane until the new lane center has been reached. The reason for this is to avoid 
"out of lane" signals from the simulator when the car spends too much time changing lane. In an early edition of this path planner, the car could cancel a lane change 
while in the process of changing lane. Reverting to the original lane was often generating an "out of lane" signal by the simulator.

# Oncoming Traffic
* The traffic in the simulator has been designed to be randomly aggressive. Often other cars do not follow safe driving rules. Sometimes they collide or even run into you. As much as possible we have to avoid collision. Sometimes it is not possible.
* As a final comment, many hours of testing show that the average time of avoiding another car deliberately crashing into you is less than 51 minutes. The probablity of survival on this loop is under an hour! Given these conditions, avoiding crash for 20 minutes is a reasonable goal.