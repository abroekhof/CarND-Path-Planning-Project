# Path Planning Project
- Alex Broekhof

## Rubric criteria

### The car is able to drive at least 4.32 miles without incident

#### The car follows the speed limit
I calculated the speed of the car by setting the spacing of the waypoints to produce 46 MPH (line 195). I did a bit less than 50 MPH as this spacing is in the s-d coordinate frame, which can result in smaller/larger spaces in the x-y frame, and the car going over the limit.

#### Max acceleration and jerk are not exceeded
- s axis: I avoided over-acceleration on the s-axis by using a speed controller. I determined the amount I could increment/decrement the spacing while staying within the acceleration limits. Then if the speed was not near the set-point, I would change the spacing by this amount until the speed matched.
- d axis: I handled changes in the d axis (primarily lane changes) by using jerk minimizing trajectories. This was done by solving a 5th order polynomial with known start and end position, velocity and acceleration, in a certain amount of time. Since a lane change, only the d position is changing, both velocity and acceleration boundary conditions can be set to zero. The resulting coefficients were stored between cycles, as the lane change spanned multiple cycles of the simulator. A counter was used to determine whether the desired amount of time elapsed, and to end the lane change.

#### Car does not have collisions
Sensor data, showing the position and velocity of other cars on the road was used to detect the situation in front of the car. If a vehicle was detected within a certain range (line 215), the speed of that lane is set to the speed of that car. Also, cars are detected on either side, to determine whether it is safe to change lanes (line 225). With lane speed information it is simple to find the fastest open lane which is accessible, then use the lane shifting procedure described above to move to that lane. If the lane is not accessible, and the car must follow another vehicle, the target speed is set to the speed of the leading car. Furthermore, if the distance between the cars becomes too small, the target speed is decreased further until it is safe.

#### The car stays in its lane, except for the time between changing lanes
To ensure that the car stays in its lane, I created splines which mapped from the s-axis to the x, y, dx and dy axes. By keeping track of the s position of the car, I could easily compute the desired x-y coordinates (line 299). As the lane size is 4m, the lane position could be calculated as 2+4*(lane index). 

#### The car is able to change lanes
This is described above.

## Reflection
This was a very interesting project, particularly in seeing how difficult it is to make general rules for driving, even in the relatively simple world of the simulator. Also, watching the car drive itself, I realized how much planning we are able to do. For example, humans easily know when it makes sense to move into a lane where there is a slower vehicle, but would allow for an immediate pass. This is certainly programmable, but it is one of many corner cases which must be accounted for.