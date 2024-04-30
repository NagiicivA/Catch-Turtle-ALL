import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import random

class TurtleSimController(Node):
    def __init__(self):
        super().__init__('turtle_sim_controller')
        self.publisher = {}
        self.publisher['turtle1'] = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        for i in range(1, 100):
            self.publisher['turtle'+str(i+1)] = self.create_publisher(Twist, 'turtle'+str(i+1)+'/cmd_vel', 10)
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.turtles = {'turtle1': Pose()}
        self.turtle_chain = ['turtle1']  # List to track turtles in the chain
        self.timer = self.create_timer(3.0, self.spawn_turtle)
        self.create_subscription(Pose, 'turtle1/pose', lambda msg: self.update_turtle_pose(msg, 'turtle1'), 10)

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = random.uniform(2, 8)
        request.y = random.uniform(2, 8)
        request.theta = random.uniform(0, 2 * math.pi)
        turtle_name = f'turtle{len(self.turtles) + 1}'
        request.name = turtle_name
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda future: self.handle_turtle_spawned(future, turtle_name))

    def handle_turtle_spawned(self, future, name):
        response = future.result()
        if response and response.name == name:
            self.turtles[name] = Pose()
            self.create_subscription(Pose, f'{name}/pose', lambda msg: self.update_turtle_pose(msg, name), 10)

    def update_turtle_pose(self, msg, name):
        self.turtles[name] = msg

    def find_nearest_turtle(self):
        master_pose = self.turtles['turtle1']
        nearest_turtle = None
        min_distance = float('inf')
        for name, pose in self.turtles.items():
            if name == 'turtle1' or name in self.turtle_chain:
                continue
            distance = self.calculate_distance(master_pose, pose)
            if distance < min_distance:
                min_distance = distance
                nearest_turtle = name
        return nearest_turtle

    def calculate_distance(self, pose1, pose2):
        return math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)

    def navigate_to_turtle(self, target_name):
        while rclpy.ok():
            target_pose = self.turtles[target_name]
            current_pose = self.turtles['turtle1']
            angle_to_target = math.atan2(target_pose.y - current_pose.y, target_pose.x - current_pose.x)
            distance_to_target = self.calculate_distance(current_pose, target_pose)

            twist = Twist()
            # Calculate the angular velocity
            angle_difference = angle_to_target - current_pose.theta
            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi
            twist.angular.z = angle_difference * 2

            # Move forward if the orientation is roughly correct
            if abs(angle_difference) < 0.1:
                twist.linear.x = min(0.5, distance_to_target)

            self.publisher['turtle1'].publish(twist)

            # Check if the turtle is close enough to consider 'caught'
            if distance_to_target < 0.5:
                self.turtle_chain.append(target_name)  # Add the turtle to the chain
                print(f"{target_name} caught and added to the chain.")
                break  # Exit loop since the target is caught
            self.follow_leader()

            rclpy.spin_once(self, timeout_sec=0.1)
    def follow_leader(self):
    # Loop through each turtle in the chain except the first (leader)
            if len(self.turtle_chain)>1 :
       
               for i in range(1, len(self.turtle_chain)):
                   leader_name = self.turtle_chain[i-1]
                   follower_name = self.turtle_chain[i]
                   leader_pose = self.turtles[leader_name]
                   follower_pose = self.turtles[follower_name]

        # Compute the angle to the leader
                   angle_to_leader = math.atan2(leader_pose.y - follower_pose.y, leader_pose.x - follower_pose.x)
        # Compute distance to the leader
                   distance_to_leader = self.calculate_distance(leader_pose, follower_pose)

                   twist = Twist()
        # Adjust angular velocity
                   angle_difference = angle_to_leader - follower_pose.theta
                   if angle_difference > math.pi:
                      angle_difference -= 2 * math.pi
                   elif angle_difference < -math.pi:
                      angle_difference += 2 * math.pi
                   twist.angular.z = 2 * angle_difference  # Increase the angular velocity multiplier for faster correction

        # Adjust linear velocity to maintain a distance of 1 unit
                   if abs(angle_difference) < 0.2:  # Reduced threshold for smoother following
                       twist.linear.x = max(0.5, 0.5 * (distance_to_leader - 1))  # Ensure positive speed value and adjust proportionally
            # if len(self.turtle_chain)>1 :
            #     for i in range(1, len(self.turtle_chain)):
            #         self.publisher = self.create_publisher(Twist, 'turtle'+str(i+1)+'/cmd_vel', 10)
                   self.publisher[follower_name].publish(twist)

    def run_simulation(self):
        while rclpy.ok():
            nearest_turtle = self.find_nearest_turtle()
            if nearest_turtle:
                self.navigate_to_turtle(nearest_turtle)
            rclpy.spin_once(self, timeout_sec=0.1)
    def simple_move(self):
        twist = Twist()
        twist.linear.x = 0.5  # Move forward at a constant speed
        twist.angular.z = 0.0  # No rotation
        self.publisher['turtle2'].publish(twist)

def main():
    rclpy.init()
    turtle_controller = TurtleSimController()
    turtle_controller.run_simulation()

if __name__ == '__main__':
    main()