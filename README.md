# Motion-planning-for-an-Autonomous-Vehicles-under-Dynamic-Obstacle-Uncertainty
This project is regarding Motion Planning for Autonomous Vehicle under uncertainty. 
There are various kinds of uncertainty for an autonomous vehicle/autonomous robot like: Uncertainty in system configuration, Uncertainty in the system model, Uncertainty in environmental situations​,  Uncertainty in Future environment state.(Dynamic obstacle)​.
So, as to deal with these uncertainties, motion planner should plan by incorporating these uncertainties. We majorly focus on dynamic obstacle uncertainty.  After doing literature survey, we found that sampling-based approach scale well with respect to deal with uncertainty. 
So, we studied and explored following RRT variants which are use to deal with uncertainty: 
RRBT(Rapidly exploring Belief trees)
​Chance Constrained RRT(CC-RRT)
​ RRT-FNDynamic()
​ RRT-X We have successfully implemented RRT for Ackerman based vehicle for static obstacle.(You can see the implementation in the below video)  We finally implemented RRT* as global planner and Dynamic Window approach algorithm as local planner. 
RRT* returned us a optimal path, we fed the path to our local planner. 
We simulated our dynamic moving obstacle in Pygame by incorporating random uncertain moving obstacles. 
Our local planner is successfully able to avoid uncertain moving dynamic obstacles and it reached it's goal destination without colliding to any of the obstacles.
