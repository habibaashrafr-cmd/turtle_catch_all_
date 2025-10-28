import math
import random
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.counter = 2
        self.alive_turtles_ = []
        self.alive_turtl_pub_ = self.create_publisher(TurtleArray, "/alive_turtles",10)
        self.spawn_clinet_ = self.create_client(Spawn, "/spawn")
        self.kill_clinet_ = self.create_client(Kill, "/kill")
        self.spwan_timer_ = self.create_timer(0.5, self.spwan_new_turtle)
        self.catch_turtle_service_ = self.create_service(
            CatchTurtle, "/catch_turtle",self.callback_catch_turtle)

    def callback_catch_turtle(self, request :CatchTurtle.Request, response: CatchTurtle.Response ):
        self.call_kill_service(request.name)
        response.success = True
        return response

    def call_kill_service(self, name):
        while not self.kill_clinet_.wait_for_service(1.0):
            self.get_logger().warn("wait for kill service...")

        request = Kill.Request()
        request.name = name 

        future = self.kill_clinet_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill_service, name = name))

    def callback_call_kill_service(self, future, name):
        for i,turtle in enumerate(self.alive_turtles_):
            if turtle.name == name :
                del self.alive_turtles_[i]
                self.pub_alive_turtle()
                break

    def spwan_new_turtle(self):
        self.counter +=1
        turtle_name = "turtle" + str(self.counter)
        x = random.uniform(0.0,11.0)
        y = random.uniform(0.0,11.0)
        theta = random.uniform(0.0, 2* math.pi)
        self.call_server_spawn(x, y, theta, turtle_name)

    def pub_alive_turtle(self):
        msg = TurtleArray()
        msg.turtels = self.alive_turtles_
        self.alive_turtl_pub_.publish(msg)

    def call_server_spawn(self,x, y, theta, turtle_name):
        while not self.spawn_clinet_.wait_for_service(1.0):
            self.get_logger().warn("wait for spawn service...")
        request = Spawn.Request()
        request.x = x 
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.spawn_clinet_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spwan_service, request = request))

    def callback_call_spwan_service(self, future, request):
        response : Spawn.Response = future.result()
        if response.name != "" :
            self.get_logger().info("New alive turtle " + response.name)
            new_turtle_ = Turtle()
            new_turtle_.name = response.name
            new_turtle_.x = request.x
            new_turtle_.y = request.y
            new_turtle_.theta = request.theta
            self.alive_turtles_.append(new_turtle_)
            self.pub_alive_turtle()
            
        


def main(args=None):
    rclpy.init(args = args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__" :
    main()
