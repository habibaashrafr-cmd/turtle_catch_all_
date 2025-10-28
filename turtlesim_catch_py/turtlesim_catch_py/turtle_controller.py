import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from functools import partial
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.turtle_to_catch : Turtle = None
        self.pose_: Pose = None
        # self.closest_turtle_
        self.alive_turtle_sub_ = self.create_subscription(
            TurtleArray, "/alive_turtles", self.callback_alive_turtles, 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
        self.cmd_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.control_time_ = self.create_timer(0.01, self.pub_cmd)
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "/catch_turtle")

    def callback_call_CatchTurtle_service(self, future, name):
        response : CatchTurtle.Response = future.result()
        if not response.success :
            self.get_logger().error("Turtle " + name + " could not be removed")


    def call_catch_turtle_service(self,name):
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("wait for catch_turtle service...")
        
        request = CatchTurtle.Request()
        request.name = name

        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_CatchTurtle_service, name = name))


    def callback_alive_turtles(self, live_turtle: TurtleArray):
        if len(live_turtle.turtels) > 0 :
            closest_turtle = None
            closest_turtle_distance = None
            for i in live_turtle.turtels :
                dist_x = i.x - self.pose_.x
                dist_y =i.y - self.pose_.y
                close_dis = math.sqrt(dist_x**2 + dist_y**2)
                if closest_turtle == None or close_dis < closest_turtle_distance :
                    closest_turtle_distance = close_dis
                    closest_turtle = i
                self.turtle_to_catch = closest_turtle

            # self.turtle_to_catch = live_turtle.turtels[0]


    def callback_pose(self, Pose: Pose):
        self.pose_ = Pose

    def pub_cmd(self):
        if self.pose_ == None or self.turtle_to_catch == None :
            return
        
        cmd = Twist()
        dist_x = self.turtle_to_catch.x -self.pose_.x 
        dist_y = self.turtle_to_catch.y - self.pose_.y 
        distance = math.sqrt(dist_x**2 + dist_y**2)
        
        if distance > .5 :
            cmd.linear.x = 2*distance

            theta = math.atan2(dist_y, dist_x)
            dif_theta = theta - self.pose_.theta 
            if dif_theta > math.pi :
                dif_theta -= 2*math.pi
            elif dif_theta < -math.pi :
                dif_theta += 2*math.pi
            cmd.angular.z = 6*dif_theta
        else :
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch.name)
            self.turtle_to_catch = None
        
        self.cmd_publisher_.publish(cmd)



def main(args=None):
    rclpy.init(args = args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__" :
    main()
