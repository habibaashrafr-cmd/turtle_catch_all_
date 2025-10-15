#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals ;

class TurtleControllerNode :public rclcpp::Node 
{
public :
    TurtleControllerNode() : Node("turtle_controller"), turtlesim_up_(false), catch_closest_turtle_first_(true)
    {
        catch_turtle_client_ = this->create_client<my_robot_interfaces::srv::CatchTurtle>("/catch_turtle") ;
        alive_turtles_subscrip_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>
            ("/alive_turtles", 10, std::bind(&TurtleControllerNode::callbackAliveTurtles, this, _1)) ;
        turtle_pose_ = this->create_subscription<turtlesim::msg::Pose> ("/turtle1/pose", 10, 
                       std::bind(&TurtleControllerNode::callbackTurtlepose, this, _1)) ;
        turtle_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist> ("/turtle1/cmd_vel", 10) ;               
        control_loob_timer_ = this->create_wall_timer(0.001s, std::bind(&TurtleControllerNode::ControLoop, this)) ;  
    }
private :
 double getDistanceFromCurrentPose(my_robot_interfaces::msg::Turtle turtle)
    {
        double dist_x = turtle.x - pose_.x;
        double dist_y = turtle.y - pose_.y;
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    void callCatchTurtleService(std::string turtle_name)
    {
        while (!catch_turtle_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }
        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>() ;
        request->name = turtle_name ;

        catch_turtle_client_->async_send_request(request, 
            std::bind(&TurtleControllerNode::callbackCallCatchTurtleService, this, _1)) ;
        
    }

    void callbackCallCatchTurtleService(rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedFuture future)
    {
        auto response = future.get() ;
        if (!response->success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to remove turtle") ;
        }
    }

    void callbackTurtlepose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        pose_ = *pose.get() ;
        turtlesim_up_ = true ;
    }

    void callbackAliveTurtles(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        if (!msg->turtels.empty())
        {
            if (catch_closest_turtle_first_)
            {
                my_robot_interfaces::msg::Turtle closest_turtle = msg->turtels.at(0);
                double closest_turtle_distance = getDistanceFromCurrentPose(closest_turtle);

                for (int i = 1; i < (int)msg->turtels.size(); i++)
                {
                    double distance = getDistanceFromCurrentPose(msg->turtels.at(i));
                    if (distance < closest_turtle_distance)
                    {
                        closest_turtle = msg->turtels.at(i);
                        closest_turtle_distance = distance;
                    }
                }

                turtle_to_catch_ = closest_turtle;
            }
            else
            {
                turtle_to_catch_ = msg->turtels.at(0);
            }
        }
    }


    void PublishComdVel(double x, double theta)
    {
        auto msg = geometry_msgs::msg::Twist() ;
        msg.linear.x = x ;
        msg.angular.z = theta ;
        turtle_cmd_vel_->publish(msg) ;
    }

    void ControLoop()
    {
        if (!turtlesim_up_ || turtle_to_catch_.name == "")
        {
           return ;
        }
        double dist_x = turtle_to_catch_.x - pose_.x ;
        double dist_y = turtle_to_catch_.y - pose_.y ;
        double dist_ = sqrt(dist_x * dist_x + dist_y * dist_y) ;
         auto msg = geometry_msgs::msg::Twist() ;
        if (dist_ > 0.5)
        {
            msg.linear.x = 2*  dist_ ;
            double target_angle_ = atan2(dist_y, dist_x) ;
            double angle_ = target_angle_ - pose_.theta ;
            if (angle_ > M_PI)
                angle_ -= 2*M_PI ;
            else if (angle_ < -M_PI)
                 angle_ += 2*M_PI ;
            msg.angular.z = 6 * angle_ ;     
        }
        else
        {
            msg.linear.x = 0.0 ;
            msg.angular.z = 0.0 ;
            callCatchTurtleService(turtle_to_catch_.name) ;
            turtle_to_catch_.name = "" ;
        }
        turtle_cmd_vel_->publish(msg) ;
    }

    bool turtlesim_up_, catch_closest_turtle_first_ ;
    turtlesim::msg::Pose pose_ ;
    my_robot_interfaces::msg::Turtle turtle_to_catch_ ;

    rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_client_ ;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscrip_ ;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  turtle_cmd_vel_ ;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_ ;
    rclcpp::TimerBase::SharedPtr control_loob_timer_ ;
}; 


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv) ;
    auto node = std::make_shared<TurtleControllerNode>() ;
    rclcpp::spin(node) ;
    rclcpp::shutdown() ; 
    return 0;
}