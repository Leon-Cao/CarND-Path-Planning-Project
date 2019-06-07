# CarND-Path-Planning-Project

[//]: # (Image References)

[image1]: ./result.png "Safe Drive Result"


## Rubrics 

### The code compiles correctly. (Code must compile without errors with cmake and make.)
  * I created build folder and used 'cmake ..' to generate Makefile. 
  * There is no any error and warning when execute 'make'

### The car is able to drive at least 4.32 miles without incident..
  * Please check below image to for the safe drive result.
  ![alt text][image1]
  
### The car drives according to the speed limit.
  * Speed limited on (49.9 - accelerate). Then even car is running on max accelerate velocity, the car's speed still less than 50MPH

### Max Acceleration and Jerk are not Exceeded.
  * Macro NORMAL_ACCELERATE_VEL defined for normal acceleration. And the value is 0.3 almost equal to 3m/s^2.
  * The Max Acceleration may happenned on there is car in back and it is too close to self (diff_s<10). Then self drive car will use double NORMAL_ACCELERATE_VEL to avoid collision. This secnario always happen on changing lane.
  * Macro NORMAL_JERK is defined for Jerk and it equals to 0.224 almost same as 2m/s^2.  

### Car does not have collisions.
  * Yes, there is no collisions.

### The car stays in its lane, except for the time between changing lanes.
  * Yes, The car can stay in its lane, except changing lane.

### The car is able to change lanes
  * Yes, the car can change lanes without any issues. 

### There is a reflection on how to generate paths.
  * 

