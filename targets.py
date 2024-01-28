import rclpy
from turtlesim.msg import Pose
import turtlesim.srv
from random import randint

def spawn_target():
    # Create a service client to spawn a new target turtle
    node = rclpy.create_node('spawn_target_client')
    client = node.create_client(turtlesim.srv.Spawn, '/spawn')

    # Wait for the service to be available
    if not client.service_is_ready():
        client.wait_for_service()

    # Create the request message
    request = turtlesim.srv.Spawn.Request()
    request.x = float(randint(1,9))
    request.y = float(randint(1,9))
    request.theta = 0.0
    request.name = 'target'

    # Call the service
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # Check if the service call was successful
    if future.result() is not None:
        print("Target turtle spawned successfully")
    else:
        print("Failed to spawn target turtle")

    # Cleanup
    node.destroy_client(client)
    
def destroy_turtle():
    # Create a service client to kill the current turtle
    node = rclpy.create_node('destroy_turtle_client')
    client = node.create_client(turtlesim.srv.Kill, '/kill')

    # Wait for the service to be available
    if not client.service_is_ready():
        client.wait_for_service()

    # Create the request message
    request = turtlesim.srv.Kill.Request()
    request.name = 'target'  # The default turtle name in turtlesim

    # Call the service
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # Check if the service call was successful
    if future.result() is not None:
        print("Current turtle destroyed successfully")
    else:
        print("Failed to destroy current turtle")

    # Cleanup
    node.destroy_client(client)

def pose_callback(data):
    # Check for collision with the target
    target_x = float(randint(1,9))
    target_y = float(randint(1,9))
    collision_threshold = 0.5

    distance_to_target = ((data.x - target_x)**2 + (data.y - target_y)**2)**0.5

    if distance_to_target < collision_threshold:
        print("Collision with target!")
        destroy_turtle()  # Destroy the current turtle
        spawn_target()  # Respawn the target

def turtle_listener():
    rclpy.init()
    node = rclpy.create_node('turtle_collision_listener')
    subscriber = node.create_subscription(Pose, '/turtle1/pose', pose_callback, 10)

    # Spawn the initial target turtle
    spawn_target()

    while rclpy.ok():
        rclpy.spin_once(node)

if __name__ == '__main__':
    turtle_listener()
