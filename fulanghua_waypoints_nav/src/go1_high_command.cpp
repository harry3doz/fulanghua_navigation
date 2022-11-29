#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <fulanghua_action/special_moveAction.h>
#include <camera_action/camera_pkgAction.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include <unitree_legged_msgs/HighCmd.h>

typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;

int main(int argc, char** argv){
  ros::init(argc, argv, "go1_high_command");
  ros::NodeHandle nh;
  ros::Rate rate(10.0); // set the rate 
  ros::Publisher go1_ros_cmd_pub; 
  Server server(nh, "go1_command", false); //make a server
  unitree_legged_msgs::HighCmd high_cmd_ros;
  std::string cmd[4] = {"standup", "sitdown", "rightsidestep", "leftsidestep"};
  std::string _high_cmd;
  ros::Time start_time;
  ros::NodeHandle private_nh("~"); 
  private_nh.param("high_cmd", _high_cmd, std::string("high_cmd"));
  go1_ros_cmd_pub=nh.advertise<unitree_legged_msgs::HighCmd>(_high_cmd, 10);
  fulanghua_action::special_moveGoalConstPtr current_goal; // instance of a goal
  server.start(); //start the server
  while (ros::ok()){
      if(server.isNewGoalAvailable()){
          current_goal = server.acceptNewGoal();
          start_time = ros::Time::now();
      }
      if(server.isActive()){
        if(server.isPreemptRequested()){
          server.setPreempted(); // cancel the goal
          ROS_WARN("Preemmpt Goal\n");
        }else{
          if(start_time + ros::Duration(current_goal->duration) < ros::Time::now()){
            server.setAborted(); // abort it
          }
          else{
            ROS_INFO("go1 starts move");
            fulanghua_action::special_moveFeedback feedback; // set the feeback
            feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration; // decide the rate of feedback
            server.publishFeedback(feedback); //publish the feedback
            //initialize high cmd
            high_cmd_ros.head[0] = 0xFE;
            high_cmd_ros.head[1] = 0xEF;
            high_cmd_ros.levelFlag = 0x00;
            high_cmd_ros.mode = 0;
            high_cmd_ros.gaitType = 0;
            high_cmd_ros.speedLevel = 0;
            high_cmd_ros.footRaiseHeight = 0;
            high_cmd_ros.bodyHeight = 0;
            high_cmd_ros.euler[0] = 0;
            high_cmd_ros.euler[1] = 0;
            high_cmd_ros.euler[2] = 0;
            high_cmd_ros.velocity[0] = 0.0f;
            high_cmd_ros.velocity[1] = 0.0f;
            high_cmd_ros.yawSpeed = 0.0f;
            high_cmd_ros.reserve = 0;
            
            if (current_goal->command == cmd[0]){
                ROS_INFO("Go1 standing up");
                high_cmd_ros.mode = 6;
            }else if (current_goal->command == cmd[1]){
                ROS_INFO("Go1 sitting down");
                high_cmd_ros.mode = 5;
            }else if (current_goal->command == cmd[2]){
                ROS_INFO("Go1 right side stepping");
                high_cmd_ros.velocity[1] = 0.112f;
            }else if (current_goal->command == cmd[3]){
                ROS_INFO("Go1 left side stepping");
                high_cmd_ros.velocity[1] = -0.112f;
            }

            while(start_time + ros::Duration(current_goal->duration) -ros::Duration(0.5) > ros::Time::now()){
              go1_ros_cmd_pub.publish(high_cmd_ros);
              rate.sleep();
            }
          }
        }
      }
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}