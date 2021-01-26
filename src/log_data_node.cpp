#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <log_data/Recovery.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <cmath>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <typeinfo>
#include <string.h>
#include <stdlib.h>
#include "boost/date_time/posix_time/posix_time.hpp"


//Initialisation

float old_x = 0;
float old_y = 0;
float current_x;
float current_y;
float meters = 0;
float old_meters = 0;
float goal_x;
float goal_y;

ros::WallTime started_time;
ros::WallTime current_time;
ros::Subscriber goal_id_sub;
ros::Subscriber recovery_sub;
ros::Subscriber amcl_pose_sub;
ros::Publisher distance_pub;
ros::Publisher time_nav;
ros::Publisher recovery_pub;
log_data::Recovery bot_state;
std_msgs::Float64 dist;
std_msgs::Header time_nav_msg;

//Methods 

float calculateDistance(float current_x, float current_y,float old_x, float old_y)

{
    return sqrt((pow(current_x - old_x, 2) + pow(current_y - old_y, 2)));
}

void setOldposition(float current_x, float current_y)
  {
      old_x = current_x;
      old_y = current_y;
  }


void poseAmclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)

{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y; 

    if (old_x != 0.0 && old_y != 0.0)
    {
      meters = calculateDistance(current_x, current_y, old_x, old_y);
      old_meters = old_meters + meters;
      //ROS_INFO("current_x = %f, current_y = %f, old_x= %f, old_y = %f, meters = %f, old_meters = %f",current_x,current_y,old_x,old_y,meters,old_meters);
      setOldposition(current_x, current_y);
    }
    else
      setOldposition(current_x, current_y);
    
    
    dist.data = meters;
    //ROS_INFO("%f",meters);
    distance_pub.publish(dist);
    meters = 0;
}

void goalIDCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& msg)
{
    goal_x = msg->goal.target_pose.pose.position.x;
    goal_y = msg->goal.target_pose.pose.position.y;

}


void RecoveryCallback(const move_base_msgs::MoveBaseActionResultConstPtr& msg)
{    
    int n = msg->status.text.size();
    char st[n + 1];
    strcpy(st,msg->status.text.c_str());
    current_time = ros::WallTime::now();
    double secs = (current_time - started_time).toSec();
    double nanosecs = (current_time - started_time).toNSec();
    if(strstr(st, "plan"))
        {   bot_state.duration_time.sec = secs;
            bot_state.duration_time.nsec = nanosecs;
            bot_state.reco_type = "Plan failed";
            bot_state.x_start = current_x;
            bot_state.y_start = current_y;
            bot_state.x_goal = goal_x;
            bot_state.y_goal = goal_y;
            recovery_pub.publish(bot_state);
        }
    else if(strstr(st, "control")){
            bot_state.duration_time.sec = secs;
            bot_state.duration_time.nsec = nanosecs;
            bot_state.reco_type = "Controller failed";
            bot_state.x_start = current_x;
            bot_state.y_start = current_y;
            bot_state.x_goal = goal_x;
            bot_state.y_goal = goal_y;
            recovery_pub.publish(bot_state);
    }
    else if(strstr(st, "oscillating")){
            bot_state.duration_time.sec = secs;
            bot_state.duration_time.nsec = nanosecs;
            bot_state.reco_type = "Oscillation occured";
            bot_state.x_start = current_x;
            bot_state.y_start = current_y;
            bot_state.x_goal = goal_x;
            bot_state.y_goal = goal_y;
            recovery_pub.publish(bot_state);
    } 
 
    
}




int main(int argc, char * argv[])
{
   ros::init(argc, argv, "log_data_node");
   ros::NodeHandle nh;
   amcl_pose_sub = nh.subscribe("/amcl_pose", 1, poseAmclCallback);
   goal_id_sub = nh.subscribe("/move_base/goal",10, goalIDCallback);
   recovery_sub = nh.subscribe("/move_base/result",10,RecoveryCallback);
   recovery_pub = nh.advertise<log_data::Recovery>("/recovery_state",1);
   distance_pub = nh.advertise<std_msgs::Float64>("/distance", 1);
   ros::Rate loop_rate(1);
   started_time = ros::WallTime::now();
   // Affichage du starting time em human stamp
    ros::WallDuration d(5*60*60);
    boost::posix_time::ptime my_posix_time = (ros::WallTime::now()-d).toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    ROS_INFO("time : %s",iso_time_str.c_str());
  //Affichage du starting time em human stamp

  while(ros::ok())
  {
  ros::spinOnce();
  loop_rate.sleep();
  }
  //ros::spin();
   return 0;
   
}






