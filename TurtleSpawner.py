import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.client = self.create_client(Spawn, 'spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.timer = self.create_timer(3.0, self.spawn_turtle)

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = random.uniform(0, 10)  # Assuming Turtlesim space is from 0 to 10
        request.y = random.uniform(0, 10)
        request.theta = random.uniform(0, 3.14)  # Random orientation
        request.name = f'turtle_{random.randint(1, 10000)}'  # Random name to avoid conflicts
        future = self.client.call_async(request)
        future.add_done_callback(self.spawn_response)

    def spawn_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Spawned turtle at [{response.x}, {response.y}] with name {response.name}')
        except Exception as e:
            self.get_logger().info(f'Failed to spawn turtle: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    turtle_spawner = TurtleSpawner()
    rclpy.spin(turtle_spawner)
    turtle_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
