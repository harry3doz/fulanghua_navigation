#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <fulanghua_action/special_moveAction.h>
#include <actionlib/server/simple_action_server.h>
#include <fulanghua_msg/_LimoStatus.h>
typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;

double battery_=0.0;
void limo_batteryCallback(const fulanghua_msg::_LimoStatus &msg){
    battery_ = msg.battery_voltage;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "battery_check_server");

  ros::NodeHandle nh;
  Server server(nh, "battery_check", false); //make a server

  double charge_threshold_higher_;
  ros::Rate rate(10.0); // set the rate 
  ros::NodeHandle private_nh("~"); 
  ros::Subscriber charge_sub;
  std::string robot_name_;
  std::string CHARGE_TOPIC;
  private_nh.param("charge_threshold_higher", charge_threshold_higher_, 12.0);
  private_nh.param("robot_name", robot_name_, std::string("go1"));
    if (robot_name_ !="go1"){
        #define LIMO
        CHARGE_TOPIC = "/limo_status";
    }else{
        CHARGE_TOPIC = "/go1_status";
    }
  ros::Time start_time;
  fulanghua_action::special_moveGoalConstPtr current_goal; // instance of a goal
    #ifdef LIMO
        charge_sub = nh.subscribe(CHARGE_TOPIC, 100, limo_batteryCallback);
    #else
        charge_sub = nh.subscribe(CHARGE_TOPIC, 100, go1_batteryCallback);
    #endif
  server.start(); //start the server
  while (ros::ok()){
      if(server.isNewGoalAvailable()){
          current_goal = server.acceptNewGoal();
          start_time = ros::Time::now();
      }
      if(server.isActive()){
        if(server.isPreemptRequested()){
          server.setPreempted(); // cancel the goal
          ROS_WARN("Battery check: Preemmpt Goal\n");
        }else{
          if(start_time + ros::Duration(current_goal->duration) < ros::Time::now()){
            server.setPreempted(); // abort it
            ROS_WARN("Battery check: Preemmpt Goal\n");
          }
          else{
            fulanghua_action::special_moveFeedback feedback; // set the feeback
            feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration; // decide the rate of feedback
            server.publishFeedback(feedback); //publish the feedback
            ROS_INFO("Current battery: %lf", battery_);
            ROS_INFO("Target: %lf", charge_threshold_higher_);
            if (battery_ >=charge_threshold_higher_){
                server.setSucceeded();
                ROS_INFO("Battery check: Succeeded it!");
            }else{

                ros::Duration(5).sleep();
            }
          }
        }
      }
    ros::spinOnce();
    rate.sleep();
    }
  return 0;
}