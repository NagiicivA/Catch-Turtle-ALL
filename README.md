# Catch-Turtle-ALL
An assessment from school
In this project, you are required to develop and implement an interactive simulation in 
the Turtlesim environment using ROS2. Your main objective is to program a master 
turtle to "catch" other turtles that appear randomly within the environment. This 
project aims to test your proficiency with ROS2, focusing on topics, services, and 
action clients, and your ability to devise efficient algorithms for real-time robotic 
interaction in the Turtlesim simulation environment.

Task Requirements
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

Evaluation Criteria
Turtle Catching Efficiency: The master turtle must successfully identify and move towards 
the nearest turtle.
Chain Formation: The caught turtles must successfully attach and maintain formation 
directly behind the master turtle, forming a coherent chain.
