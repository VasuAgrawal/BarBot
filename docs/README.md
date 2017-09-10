# BarBot - Drinks for the Lazy!

## Motivation
BarBot is the realization of the ultimate efficiency of resort pool bars – how can a patron receive their drink while expending minimal effort to do so? The answer is an autonomous system that allows users to place  orders from tablets spaced around the pool, and simply sit back and relax while their drink is delivered to them by a robot. 

BarBot solves multiple interesting challenges in robotics, including locomotion in water, as well as the mechanical challenge of stabilizing a drink on a moving aquatic platform. In addition, the task of localizing multiple robots and multiple patrons in an aquatic environment was a significant challenge, as the presence of water rules out most vision-based approaches. We hope our project has made significant progress towards realizing these challenges in modern robotics.

## Requirements
* __Navigation__ – BarBot must navigate the full service area of the pool. 
* __Multiple BarBots__ – Multiple robots must work together to deliver drinks to patrons in the pool.
* __Drink Management__ – BarBots must stabilize the drinks they are carrying and ensure no spillage or contamination.
* __Multiple Drinks__ – Each BarBot must be able to carry multiple drinks. 
* __Safety__ – BarBot must not pose a danger to patrons in the pool.
* __Waterproof__ – BarBot must be waterproof, particularly the electronics. 
* __Reliability__ – System must be robust to failure of any individual component, and should recover gracefully from failures. 

## Physical Design
* Iterated design from 4-thruster holonomic vacuum-formed chassis to final 2-thruster skid-steer box chassis.
* Electronics box sealed and enclosed inside robot.
* Ballast - added approximately 40lbs to robot.

<img src="/docs/Physical1.jpg" height="250"> <img src="docs/Physical2.jpg" height="250">

## Drink Stabilization
* Gimbal mechanism to stabilize a cup with a drink in it.
* Passive stabilization - the weight of the drink was enough to stabilize the mechanism, even in rough waters and fast driving maneuvers. 

<img src="/Gimbal1.jpg" height="250"> <img src="/Gimbal2.jpg" height="250">

## Ordering and Scheduling
* Web app to place orders. Customers can place orders at tablets located around the pool.
* Bartender interface to view which drinks to make and which robots to place them on.
* Scheduler server plans for the robots using incoming orders. Depending on the capacity of each robot and the number of robots, it will schedule robots to deliver drinks. 

<img src="/Ordering1.jpg" height="250"> <img src="/Ordering2.jpg" height="250">

## Localization
* Beacon-based time-of-flight localization system using UWB radio frequencies.
* Decawave DWM1000 modules allow us to compute distances between beacons. 
* We compute relative positions of beacons with gradient descent.
* Each robot and patron in the pool has a wristband - perform triangulation to localize robots and patrons in the pool. 
* Used a power-over-ethernet solution to ensure a robust beacon network.

<img src="/GLS1.jpg" height="250"> <img src="/GLS2.jpg" height="250">
<img src="/GLS3.jpg" height="250">

For a more detailed look into our Global Localization System, please view the "Robust Multi-Agent Aquatic Localization.pdf" file in the top-level directory of the Github repository. 

## Software Architecture
* Distributed software architecture - multiple servers spread over devices.
* Robust to node failure - all nodes automatically reconnect to each other upon connection loss.
* Each robot is running an identical ROS workspace, with the only difference being a robot ID parameter.

![Software Architecture](/SoftwareArch.jpg "Software Architecture")

## Sensing and State Estimation
* IMU provides heading of robot calculated from magnetometer readings as deviation from Earth North.
* Localization system provides positions in 3-space, in an arbitrarily rotated coordinate basis. We perform a calibration routine to compute a projection from the Global Localization frame to the frame of the pool.
* Given IMU and Global Localization input, robot computes its pose estimate.
* We use PID control to navigate the robot to a waypoint, given its pose.

## Future Work
* Implement a payment mechanism 
* Detect when a drink has been removed
* Patron authentication for drink removal
* Path planner with integrated obstacle detection and avoidance
* Docking station for easier charging

## Libraries and Frameworks
* ROS
* Tornado
* PostgreSQL
