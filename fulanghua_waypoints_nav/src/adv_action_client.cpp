#include <ros/ros.h>
#include <fulanghua_action/testAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <regex>
#include <fulanghua_srvs/actions.h>
#include <std_srvs/Trigger.h>
std::string c;
bool STOP_ACTION = false;
struct termios save_settings;
void set_input_interactive(void)
{
  // set no-echo-back and non-blocking
  struct termios settings;
  tcgetattr(0, &save_settings);
  settings = save_settings;
  settings.c_lflag &= ~(ECHO | ICANON);
  settings.c_cc[VTIME] = 0;
  settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &settings);
  fcntl(0, F_SETFL, O_NONBLOCK);
}

void reset_input(void)
{
  tcsetattr(0, TCSANOW, &save_settings);
}
void action_service_stop_callback(fulanghua_srvs::actions::Request &request,fulanghua_srvs::actions::Response &response){
  
}

typedef actionlib::SimpleActionClient<fulanghua_action::testAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_client");
  ros::NodeHandle nh;
  ros::ServiceServer command_server;
  command_server = nh.advertiseService("command_send", action_stop_callback);
  set_input_interactive();

  Client client("task", true);

  printf("i: look up\n");
  printf(",: look down\n");
  printf("j: look left\n");
  printf("j: look right\n");
  printf("t: take photo\n");
  printf("v: watch video\n");
  printf("k: return normal\n");
  printf("c: cancel goal\n");
  printf("q: quit\n");

  int task_id = 0;
  bool initial_goal = false;
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    if (client.isServerConnected())
    {
      // char _c = getchar();
      // c{_c};
      if (c == "c")
      {
        client.cancelGoal();
        printf("publish cancel\n");
      }
      else if (c == "q")
      {
        break;
      }
      else if(std::regex_match(c,std::regex("[a-z]")) || c==",") {
        // else if(c=="i") {
        fulanghua_action::testGoal goal;
        goal.task_id = task_id;
        goal.command = c;
        task_id++;
        goal.duration = INT_MAX;
        client.sendGoal(goal);
        std::cout <<"publish command:" << goal.command;
        printf(", publish goal id:%i, duration:%f\n", goal.task_id, goal.duration);
        initial_goal = true;
      }
      if (initial_goal)
        printf("Current State: %s\n", client.getState().toString().c_str());
      if(!c.empty())
        c ="";
    }
    fflush(stdout);
    ros::spinOnce();
    loop_rate.sleep();
  }
  reset_input();
  return 0;
}