# Catch-Turtle-ALL
I want to design the robot's positioning to follow
In this project, I am required to develop and implement an interactive simulation in 
the Turtlesim environment using ROS2. My main objective is to program a master 
turtle to "catch" other turtles that appear randomly within the environment. 
Now I have completed the Task A, but I have not manage task B yet.

System Requirements: The project must be implemented using ROS2 in 
conjunction with the Turtlesim simulation.

Turtle Generation: A new turtle should be spawned in a random location every 3 
seconds.

Master Turtle Behavior: The master turtle should autonomously navigate towards 
and catch the nearest available turtle.

Turtle Chain Formation: Once a turtle is caught, it should automatically follow 
directly behind the master turtle, effectively joining a growing chain of turtles.

Simulation Loop: The nodes should run in an endless loop, and the entire 
simulation should be controllable via a launch file to start or stop the simulation.

Sub-tasks Breakdown
Sub-task A: Implement the functionality to spawn a new turtle at a 
random location every 3 seconds.

Sub-task B: Develop and integrate the autonomous path planning for the 
master turtle to catch the nearest turtle. This includes dynamic decision-making as 
new turtles spawn. NB: The turtle's actions only support left turn, right turn, and 
forward and backward movement, so the turtle's behaviours need to be planned 
according to the destination.

Sub-task C: Program the behaviour for the caught turtles to follow the 
master turtle in a chain formation, maintaining close proximity to the turtle directly in 
front of them in the chain.
Turtle generation has been completed so far, but task B has not yet been completed

Evaluation Criteria
Turtle Catching Efficiency: The master turtle must successfully identify and move towards 
the nearest turtle.
Chain Formation: The caught turtles must successfully attach and maintain formation 
directly behind the master turtle, forming a coherent chain.
