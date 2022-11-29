#include <ros/ros.h>
 // Include opencv2

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <fulanghua_action/special_moveAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionServer<fulanghua_action::special_moveAction> Server;
struct timespec timer_start, timer_stop;
double fstart, fstop;

class ADJUST_POSITION{
    public:
        ADJUST_POSITION():
        rate(10),
        server(nh, "ar_detect", false),
        rotate_client("rotate", true)
        {
            ros::NodeHandle private_nh("~"); 
            private_nh.param("Kp", Kp, 5.0);
            private_nh.param("image_topic", IMAGE_TOPIC, std::string("/camera/color/image_raw"));
            private_nh.param("cmd_vel", CMD_VEL_TOPIC, std::string("/cmd_vel"));
            private_nh.param("adjust_speed", adjust_speed, 0.1);
            private_nh.param("holonomic", holonomic_, true);
            private_nh.param("offset_fixed_x", fixed_x, -0.075);
            private_nh.param("offset_fixed_y", fixed_y, -0.04);
            private_nh.param("offset_fixed_z", fixed_z, 0.37);
            private_nh.param("calibration_path", CALIBRATION, std::string(""));
            image_sub = nh.subscribe(IMAGE_TOPIC, 1000, &ADJUST_POSITION::image_callback,this);
            cmd_vel_pub =nh.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC, 10);
            cv::FileStorage fs;
            fs.open(CALIBRATION, cv::FileStorage::READ); 
            if (!fs.isOpened())
            {
                std::cout << "Failed to open " << CALIBRATION << std::endl;
            }
            fs["camera_matrix"] >> camera_matrix;
            fs["distortion_coefficients"] >> dist_coeffs;
            std::cout << "camera_matrix\n" << camera_matrix << std::endl;
            std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

                    server.start(); //start the server
                }

        void image_callback(const sensor_msgs::ImageConstPtr& msg){
            std_msgs::Header msg_header = msg->header;
            std::string frame_id = msg_header.frame_id.c_str();
            // ROS_INFO_STREAM("New Image from " << frame_id);

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            src = cv_ptr->image;
            w = src.size().width;
            h = src.size().height;
            // cv::line(src, cv::Point(w/3,0), cv::Point(w/3,h), cv::Scalar(255, 0, 0), thickness, cv::LINE_8);
            // cv::line(src, cv::Point(w/2,0), cv::Point(w/2,h), cv::Scalar(255, 255, 0), thickness, cv::LINE_8);
        }



        bool adjustPosition(double &x, double &y, double &z, double &ang){
            double t = (ros::Time::now() - start_time).toSec();
            threshold_z = (2 + t*0.05)*0.001;//0.002;

            offset_x = (double)fixed_x - x;
            offset_y = (double)fixed_y - y;
            offset_z = (double)fixed_z - z;
            ROS_INFO("start approching");
            double move_y = Kp*offset_y;
            double move_z = Kp*offset_z;
            // twist.linear.x = -move_z; // depth
            if(holonomic_)
              twist.linear.y = move_z;
            else
              twist.linear.x = -move_z;
            // twist.angular.z = move_y*15; // horizontal
            // twist.linear.y = move_x; // horizontal 
            // twist.linear.z = move_y; // vertical
            clock_gettime(CLOCK_MONOTONIC, &timer_start); fstart=(double)timer_start.tv_sec + ((double)timer_start.tv_nsec/1000000000.0);

            if(std::abs(offset_z)<=threshold_z){
                  Done_z = true;
            }
            if(Done_z){
              // Done_z = false;
              //move it to the center
              double angle=0;
              if(holonomic_){
                if (edgeScreen(3,1,2)){
                  //get the angle here
                  if (_counter>40){
                      std::sort(angle_array.begin(), angle_array.end());
                      angle = angle_array[angle_array.size()/2-1] * 0.6; // because it is too much              
                      _counter = -1;
                    }else if (_counter>=0){
                      ROS_INFO("getting an angle");
                      angle_array.push_back(ang);
                      _counter++;
                      return false;
                    }
                    //pass the angle to the rotation server
                    rotate_action(angle);
                    return true;
                }else{
                  return false;
                }
              }else{
                return true;
              }
            }else{
              ROS_INFO("Offset_z: %lf,\n", offset_z);
              ROS_INFO("Linear_y: %lf,\n", twist.linear.y);
              ROS_INFO("threshold: %lf,\n", threshold_z);
              ROS_INFO("Agnle: %lf,\n", ang);
              cmd_vel_pub.publish(twist);
              return false;
            }
              
        }

        bool edgeScreen(const double &denominator, const double &numerator_1, const double &numerator_2){
            c_x = (corners[0][0].x + corners[0][1].x)/2;
            c_y = (corners[0][0].y + corners[0][3].y)/2;
            // cv::circle(src, cv::Point(c_x,c_y),30, cv::Scalar(0, 255, 0), thickness);//Using circle()function to draw the line//
            geometry_msgs::Twist twist;
            if(c_x <(numerator_1*w/denominator)){
              twist.linear.x = adjust_speed*0.5;
            }else if (c_x> (numerator_2*w/denominator)){
              twist.linear.x = -adjust_speed*0.5;
            }else{
              return true;
            }
            cmd_vel_pub.publish(twist);
            return false;
        }


        bool rotate_action(double &angle_){
          //rotate the robot so that realsense can see it (best effort)
          if (rotate_client.isServerConnected()){
            fulanghua_action::special_moveGoal current_goal;
            current_goal.duration = 20;
            current_goal.angle = angle_;
            rotate_client.sendGoal(current_goal);
            rotate_client.waitForResult();
            ros::Duration(1).sleep();
          }
        }

        void put_commnets(double &x, double &y, double &z, double &_angle){
              vector_to_marker.str(std::string());
              vector_to_marker << std::setprecision(4)
                                << "x: " << std::setw(8) << z;
              cv::putText(src, vector_to_marker.str(),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)
                                << "y: " << std::setw(8) << y;
            cv::putText(src, vector_to_marker.str(),
                        cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)
                                << "z: " << std::setw(8) << z;
            cv::putText(src, vector_to_marker.str(),
                        cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);
            
            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4)
                                << "angle: " << std::setw(8) << _angle;
            cv::putText(src, vector_to_marker.str(),
                        cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 252, 124), 1, CV_AVX);

      }



    
    ros::NodeHandle nh;

    //publishers and  subscribers



    actionlib::SimpleActionServer<fulanghua_action::special_moveAction> server;//make a server
    actionlib::SimpleActionClient<fulanghua_action::special_moveAction> rotate_client; //for rotation client
    cv::Mat src,camera_matrix, dist_coeffs;
    double Kp; // proportional coefficient
    double Kv; // derivative coefficient
    double target; // target angle(radian)
    double offset_x =0; double offset_y=0; double offset_z=0;
    bool Done_x = false; bool Done_y = false; bool Done_z = false; bool Done_r= false;
    double fixed_x,fixed_y, fixed_z;
    double threshold_x, threshold_y, threshold_z;
    double c_x= 0.0; double c_y=0.0; double w=0.0; double h=0.0; //in case the marker is on the edge
    double adjust_speed;
    double _counter=0; std::vector<double> angle_array; //for rotation
    
    int thickness =2;
    std::string CMD_VEL_TOPIC; // target cmd_vel
    std::string IMAGE_TOPIC; // image topic 
    ros::Rate rate; // set the rate 
    geometry_msgs::Twist twist; 
    ros::Time start_time;
    
    fulanghua_action::special_moveGoalConstPtr current_goal; // instance of a goal
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50); 
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<int> ids;
    std::ostringstream vector_to_marker;
    

  private:
      ros::Publisher cmd_vel_pub;
      ros::Subscriber image_sub;
      std::string CALIBRATION;
      bool holonomic_;

};



int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_detect_server");
  ADJUST_POSITION adj;
  bool visualize = true;
  while (ros::ok()){
      if(adj.server.isNewGoalAvailable()){
          adj.current_goal = adj.server.acceptNewGoal();
          adj.start_time = ros::Time::now();
          adj._counter=0;
          visualize = true;
          adj.Done_z=false;
      }
      if(adj.server.isActive()){
        if(adj.server.isPreemptRequested()){
          adj.server.setPreempted(); // cancel the goal
          ROS_WARN("AR detect: Preemmpt Goal\n");
        }else{
          if(adj.start_time + ros::Duration(adj.current_goal->duration) < ros::Time::now()){
            adj.server.setAborted(); // abort it
          }
          else{
            if (!adj.src.empty()){        
              cv::aruco::detectMarkers(adj.src, adj.dictionary, adj.corners, adj.ids); //detecting a marker
              std::vector<cv::Vec3d> rvecs, tvecs;
              cv::aruco::estimatePoseSingleMarkers(adj.corners, 0.05, adj.camera_matrix, adj.dist_coeffs, rvecs, tvecs); //gettting x,y,z and angle
              if (adj.ids.size()>0){
                  cv::drawFrameAxes(adj.src, adj.camera_matrix, adj.dist_coeffs, rvecs[0], tvecs[0], 0.1); //drawing them on the marker
                  double _angle = rvecs[0](2)*180/M_PI;
                  //putting texst on src
                  adj.put_commnets(tvecs[0](0), tvecs[0](1), tvecs[0](2), _angle);
                  if(adj.edgeScreen(3,1,2)){
                    if(adj.adjustPosition(tvecs[0](0), tvecs[0](1), tvecs[0](2), _angle)){ //adjusting the positiion of the mobile robot
                      adj.server.setSucceeded();
                      ROS_INFO("AR detect: Succeeded!");
                      visualize = false;
                    }
                  }
              }
              cv::imshow("src", adj.src);
              cv::waitKey(3); 
              if (!visualize)
                cv::destroyAllWindows();
            }
          }
        }
      }
      ros::spinOnce();
      adj.rate.sleep();
  }
  return 0;
};
