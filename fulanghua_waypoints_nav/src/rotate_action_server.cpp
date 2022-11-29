#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <fulanghua_action/special_moveAction.h>
#include <actionlib/server/simple_action_server.h>
typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle nh;
  Server server(nh, "rotate", false); //make a server

  double Kp; // proportional coefficient
  double target; // target angle(radian)
  std::string cmd_vel_; // target cmd_vel
  ros::Rate rate(10.0); // set the rate 
  ros::NodeHandle private_nh("~"); 
  private_nh.param("Kp", Kp, 0.5);
  private_nh.param("cmd_vel", cmd_vel_, std::string("/cmd_vel"));
  ros::Publisher turtle_vel =nh.advertise<geometry_msgs::Twist>(cmd_vel_, 10);
  ros::Time start_time;
  tf::TransformListener listener;
  tf::StampedTransform target_transform;
  fulanghua_action::special_moveGoalConstPtr current_goal; // instance of a goal
  server.start(); //start the server
  while (ros::ok()){
      if(server.isNewGoalAvailable()){
          current_goal = server.acceptNewGoal();
          start_time = ros::Time::now();
          while (ros::ok()){
            try{
                listener.lookupTransform("/base_link", "/odom", ros::Time(0), target_transform);
                break;
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
         }
          //get the target angle (rotate left)
          target = current_goal->angle*3.14/180 + atan2(target_transform.getOrigin().y(),
                                  target_transform.getOrigin().x());
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
            ROS_INFO("start rotating");
            fulanghua_action::special_moveFeedback feedback; // set the feeback
            feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration; // decide the rate of feedback
            server.publishFeedback(feedback); //publish the feedback
             tf::StampedTransform current_transform;
             geometry_msgs::Point pt;
             geometry_msgs::Twist vel_msg;
            try{
              listener.lookupTransform("/base_link", "/odom",
                            ros::Time(0), current_transform);
              pt.x = current_transform.getOrigin().x();
              pt.y = current_transform.getOrigin().y();
              double current = atan2(pt.y,pt.x);
              double diff = target -current;
              printf("target: %lf\n",target);
              printf("current: %lf\n",current);
              printf("diff_from target to current_position: %lf\n",diff);

              vel_msg.angular.z =  -Kp* (target - current);
              // vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
              //                               pow(transform.getOrigin().y(), 2));
              //adjusting position bysending twist commands
              // if (std::abs(diff)<5)
                turtle_vel.publish(vel_msg);
              if(std::abs(diff)<0.1 ){
                server.setSucceeded();
                ROS_INFO("Succeeded it!");
              }
              // else if (std::abs(diff)>5){
              //   server.setPreempted();
              //   ROS_WARN("failed it...");
              // }
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
              continue;
            }
          }
        }
      }
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
};