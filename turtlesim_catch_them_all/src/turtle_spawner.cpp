#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals ;

class TurtleSpanwnerNode :public rclcpp::Node 
{
public :
    TurtleSpanwnerNode() : Node("turtle_spanwner"), turtle_counter_(0), turtle_name_("my_turtle")
    {
        catch_turtle_service_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
            "/catch_turtle", std::bind(&TurtleSpanwnerNode::callbackCatchTurtel, this, _1, _2)) ;
        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill") ;
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn") ;
        spawn_timer_ = this->create_wall_timer(0.5s, std::bind(&TurtleSpanwnerNode::spawnNewTurtle, this)) ;
        alive_turtles_publesher_ = 
            this->create_publisher<my_robot_interfaces::msg::TurtleArray>("/alive_turtles", 10) ;
    }
private :

    void publishTurtleArray()
    {
        auto msg = my_robot_interfaces::msg::TurtleArray() ;
        msg.turtels = alive_turtles_ ;
        alive_turtles_publesher_->publish(msg) ;
    }

    void callbackCatchTurtel(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                            const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response)
    {
        callKillServer(request->name) ;
        response->success = true ;
    }
    void callKillServer(std:: string name)
    {
        while (!kill_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }
        auto request = std::make_shared<turtlesim::srv::Kill::Request>() ;
        request->name = name ;
        turtle_to_remove_ = name ;

        kill_client_->async_send_request(request, std::bind(&TurtleSpanwnerNode::callbackKillServices, this, _1)) ;
    }

    void callbackKillServices(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
    {
        (void)future;
        for(int i = 0 ; i < (int)alive_turtles_.size(); i++)
        {
            if(alive_turtles_.at(i).name == turtle_to_remove_)
            {
                alive_turtles_.erase(alive_turtles_.begin() + i) ;
                publishTurtleArray() ;
                break;
            }
        }
    }

    double random()
    {
        return double(std::rand()) / (double(RAND_MAX) + 1.0) ;
    }

    void spawnNewTurtle()
    {
        turtle_counter_++ ;
        double x = random() * 10 ;
        double y = random() * 10 ;
        double theta = random() * 2 * M_PI ;
        std::string name_turtle = turtle_name_ + std::to_string(turtle_counter_) ;
        callSpwanServer(name_turtle, x, y, theta) ;
    }

    void callSpwanServer(std::string name_new_turtle, double x, double y, double theta)
    {
        while (!spawn_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...") ;
        }
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>() ;
        request->x = x ;
        request->y = y ;
        request->theta = theta ;
        request->name = name_new_turtle ;
        
        turtle_to_save_ = my_robot_interfaces::msg::Turtle() ;
        turtle_to_save_.name = name_new_turtle ;
        turtle_to_save_.x = x ;
        turtle_to_save_.y = y ;
        turtle_to_save_.theta = theta ;

        spawn_client_->async_send_request(
            request, std::bind(&TurtleSpanwnerNode::callbackSpwanServer, this, _1)) ;
    }

    void callbackSpwanServer(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        auto response = future.get() ;
        if (response->name != "")
        {
            RCLCPP_INFO(this->get_logger(), "Turtle %s is now alive", response->name.c_str()) ;
            auto new_turtle = my_robot_interfaces::msg::Turtle() ; 
            new_turtle.name = turtle_to_save_.name ;
            new_turtle.x = turtle_to_save_.x ;
            new_turtle.y = turtle_to_save_.y ;
            new_turtle.theta = turtle_to_save_.theta ;
            alive_turtles_.push_back(new_turtle) ;
            publishTurtleArray() ;
        }
    }
    int turtle_counter_ ;
    std::string turtle_name_ ;
    my_robot_interfaces::msg::Turtle turtle_to_save_ ;
    std::string turtle_to_remove_ ;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_ ;
   
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_ ;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_ ;
    rclcpp::TimerBase::SharedPtr spawn_timer_ ;
    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publesher_ ;
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_service_ ;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv) ;
    auto node = std::make_shared<TurtleSpanwnerNode>() ;
    rclcpp::spin(node) ;
    rclcpp::shutdown() ; 
    return 0;
}