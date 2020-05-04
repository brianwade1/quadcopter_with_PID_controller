# Quadcopter with PID controllers

> Quadcopter dynamics simulation with two proportional–integral–derivative (PID) controllers that adjust the motor speeds of the quadcopter and enable control of position and orientation.

![Sample Output](/images/sample_output_graph.png)

## Files

The simulation is setup, run, and controlled from Quadcopter_main. This main file calls two other class files: a quadcopter class and a PID controller class.

1. [Quadcopter_main.py](Quadcopter_main.py) - Main program
2. [Quadcopter.py](Quadcopter.py) - Quadcopter class
3. [PID_Controller.py](PID_Controller.py) - PID Controller class

## Setup and Run

The simulation is setup from within the main program ([Quadcopter_main.py](Quadcopter_main.py)). The simulation parameters are set in lines 45-59. The simulation start and end time as well as the time step are set in lines 46-48. These times are in seconds.

![sim time setup](/images/SimTimeSetup.png)

The initial position of the quadcopter (in the inertial frame) is set in lines 57-59. The pos variable is the start location in meters references to the origin of the inertial frame. The initial velocity is set with the vel variable with all velocities in meters per second and relative to the inertial frame origin. Finally, the initial angle of the quadcopter is expressed as three [Euler angles](https://en.wikipedia.org/wiki/Euler_angles) (roll, pitch, and yaw) in degrees.

![initial position](/images/InitialPosition.png)

The simulation also supports a random initial perturbation about each Euler angle. The magnitude of the perturbation is set in line 63 with the deviation variable. This is the maximum possible magnitude of the deviation in degrees per second that can occur in any of the Euler angles. The actual perturbation used in the simulation is a random draw between 0 and this value for each angle. For a simulation with no initial perturbation, the user can set the deviation to zero or comment line 65 and uncomment line 66.

![initial deviation](/images/InitialDeviation.png)

The random actual perturbations used in the simulation are written to the terminal at run time. These are the magnitude of th perturbation in the roll, pitch, and yaw directions in degrees per second. The second line is the vector addition of these angular velocities which is the total initial perturbation in degrees per second.

![Terminal Output](/images/TerminalOutput.png)

The quadcopter uses two PID controllers to command inputs for the four motor speeds. One PID controller is for position control and generates a target set of Euler angles because the quadcopter must maneuver by changing its body orientation. The second quadcopter is for orientation control and uses the target Euler angles to generate new desired motor speeds for each of the quadcopter's four motors that power its propellers. The gains for both controllers is set in lines 73-82.

![controller gains](/images/ControllerGains.png)

The first graph in the top line of the output graphics of the simulation show its 3D flight path from the start time to the end time in the upper left graph. Next, a view of the lateral position (x,y) over time looking down on the quadcopter from the inertial frame. After that, a view of the vertical position (z) over time looking from the slide of the inertial frame. Finally, the graph on the upper right of the top row shows the lateral velocity over time for each of the three internal frame axis.

The first graph of the bottom line of the output graphics show the thrust of each of the four quadcopter motors in Newtons over time. Next, the second graph shows the body torques in Newton-meters over time about each of the body's x, y, and z axis, and the third graph shows the resulting Euler angles over time in degrees. Finally, the last graph shows the angular velocity over time in degrees per second.

## Python Libraries

This simulation was built using the following python libraries and versions:

* Python v3.7.6
* numpy v1.18.1
* matplotlib v3.1.3

## References

1. Johnson, Nicholas Andrew; Control of a Folding Quadcopter with A Slung
    Load Using Input Shaping, Master's Thesis, Georgia Institute of
    Technology, May 2017
2. Gibiansky, Andrew; Quadcopter Dynamics and Simulation;
    [http://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/](http://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/)
3. Luukkonen, Teppo; Modelling and Control of a Quadcopter. Independent
    research project in applied mathematics. Aalto University.
4. CH Robotics; Understanding Euler Angles;
    [http://www.chrobotics.com/library/understanding-euler-angles](http://www.chrobotics.com/library/understanding-euler-angles)

## License

* [GNU General Public License](https://www.gnu.org/licenses/)
* Copyright (C) 2020  Brian M. Wade
