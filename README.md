# 2021 IBM Code Challenge Obstacle Avoidance System for Autonomous Vehicles

The usage of unmanned vehicles is increasing in a variety of fields including but not limited to scientific research, surveillance, agriculture and goods transfer. Along with this, the chances of accidents also increase drastically. One of the main challenges faced in autonomous vehicle development is designing techniques to navigate the vehicle from a start point to another distant waypoint while avoiding any obstacles in the environment There can be different approaches to this problem, referred to as path planning.

## How to run

1. Make sure the necessary dependencies (such as pygame module) are installed in the environment.
2. Make sure the images and the supporting python modules (rrt.py and rrtbase.py) are in the same directory.
3. Run the main.py file.

## Rapidly exploring random trees

One of the algorithms that can be used is RRT (Rapidly exploring random trees). This method involves randomly sampling waypoints to expand a tree like structure which would be used in finding a path to the destination. Since it involves random sampling, it is not required to store the complete workspace (configuration space) information and is less computationally intensive in comparison to other algorithms. The technique involves a combination of random expansion and bias towards the destination to construct a fairly efficient path. It makes sure the path or intermediate waypoints do not collide with any of the obstacles.

## Project Overview and Application plan

In a real life scenario of autonomous vehicle application, this program plays its role in between two steps:
1. Environment map imaging (or construction) 
2. Vehicle mechanics modeling.

The environment map construction involves using sensor information and computation methods to develop a global map of the configuration space (***Note**: Configuration space refers to the space which contains all the possible configuration of the vehicle i.e the workspace environment including both free space and obstacle space*). This map is required by our program, which is designed to use the obstacle and free space information within the map to construct a path plan for it using the RRT algorithm. The initial vehicle location and the destination location is required along with the constructed map.

This path plan (coordinates of the map) that the program develops can be used in the next step of the process: vehicle mechanics modeling. In this process, the calculated coordinates of the map have to be transformed to position information within the real environment and the vehicle’s velocity should be modeled accordingly so that it moves and turns its angles to stay in the path. **Note:** The initial vehicle position information has to be transformed to corresponding coordinates within the constructed map (which would require GPS information and scaling the map to the actual environment) to get the starting point of the map and the reverse process has to be done after the path coordinates are calculated. This is not handled by the program and is assumed to be taken care of in the environment map imaging and vehicle mechanics modeling processes.

The program also constructs a vehicle movement visualisation which allows the estimation of velocity (in pixels/second) and total time taken. But this is purely on the basis of simulation and the feasibility of the estimated velocity and total time will depend on how the constructed work space map is scaled to the real-life workspace environment.
### General Structure
1. Workspace map as PNG file with free space as white and obstacle space as black (The program can be easily modified to account for other colors. Its only important that there is a distinction between free and obstacle space).
2. RRT algorithm for path planning.
3. Pygame module is used for simulation.
4. Constructed path coordinates are output as a text file.
5. Constructed path image is output as PNG file.

### Limitations

1. Static environment: The program in its current state can plan paths for static environments only. To accommodate dynamic environments, continuous input of the map (with vehicle and obstacle position changes) is required. The program should be able to take this information at short time intervals and construct a new path each time. Each new path plan has to be transferred to the vehicle mechanics modeling part at these intervals so that the vehicle can update its path accordingly. This clearly means that there has to be real-time link between our path planning program and vehicle mechanics component which is not currently present.
2. The program does not calculate the vehicle velocity required to follow the constructed path. It gives an estimate of the velocity and angular requirements but that is on the basis of the simulated scenario only and that may not be compliant with real life limitations. An ideal program would model the exact velocities (and wheel configurations if necessary) required to keep the vehicle in the path. **Note:** If the vehicle follows a **Differential Drive** model, left and right velocities have to be modeled and wheel angular configuration is not required.
3. The RRT algorithm assumes the vehicle as a point source. Therefore, the obstacles have to be made larger than it's real life dimensions within the input map to account for the vehicle's width.
4. The constructed path is not necessarily the global optimum path. Since, the algorithm follows random node creation, it is possible there are shorter paths without obstacles.

## Challenges

1. We started out with Ultrasonic sensor simulation for a differential drive vehicle for obstacle avoidance but it was later determined that incorporating a path plan might not be possible in that specific state for that model.
2. Modeling vehicle velocity based on the constructed path.
