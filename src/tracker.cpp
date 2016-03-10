#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "../msg_gen/cpp/include/trackerISL/imu.h"
#include <math.h>

#include <drrobot_jaguarV2_player/MotorInfo.h>
#include <drrobot_jaguarV2_player/MotorInfoArray.h>
#include <drrobot_jaguarV2_player/RangeArray.h>
#include <drrobot_jaguarV2_player/Range.h>
#include <drrobot_jaguarV2_player/PowerInfo.h>
#include <drrobot_jaguarV2_player/StandardSensor.h>
#include <drrobot_jaguarV2_player/CustomSensor.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>

double encoder_coeff = 640;
double gyro_coeff = 813;
double gyro_drift_velocity = 0;
double gyro_drift_acceleration = 0;
double sum_th = 0;
double sum_t = 0;
double sum_t_t = 0;
double sum_t_th = 0;
double sum_dth = 0;
int count = 0;
int garbage_length = 200;
int sample_length = 2000;

bool calibrate = true ;

static const bool acquire_data = false;
static const bool print_data = true;

double speed = 0.0;
double sync_speed = 0.0;
double yaw_rate = 0.0;
double sync_yaw_rate = 0.0;
double sync_yaw_rate1 = 0.0;

void motorSensorCallback(const drrobot_jaguarV2_player::MotorInfoArray::ConstPtr& msg)
{
    int msgSize = msg->motorInfos.capacity();

    if (msgSize == 6)
    {
        speed= msg -> motorInfos[3].encoder_vel * (msg-> motorInfos[3].encoder_dir -0.5) -
               msg -> motorInfos[4].encoder_vel * (msg-> motorInfos[4].encoder_dir -0.5);
        speed = speed/encoder_coeff;
    }
}

void imuCallback(const trackerISL::imu::ConstPtr& msg){

        yaw_rate = msg->gyroz;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber encoder = n.subscribe("/drrobot_player1/drrobot_motor",1,motorSensorCallback);
  ros::Subscriber imu = n.subscribe("/imu",1,imuCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
  ros::Publisher yaw_pub = n.advertise<std_msgs::Float64>("/yaw", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double t = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  std::ofstream data;

  ros::Time current_time, last_time,starting_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  starting_time = ros::Time::now();

  ros::Rate r(30);

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    sync_yaw_rate1 = yaw_rate;

    //compute odometry in a typical way given the velocities of the robot

    double dt = (current_time - last_time).toSec();
    double t1 = (current_time - starting_time).toSec();
    //double t2 = (last_time - starting_time).toSec();
    /*double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;*/

    double delta_x =  speed * cos(th) * dt;
    double delta_y =  speed * sin(th) * dt;
    double delta_th = (sync_yaw_rate - gyro_drift_velocity - gyro_drift_acceleration * t1) * dt /gyro_coeff;



    if(calibrate){
        if(count == 0){
            std::cout<<"Calibrating gyro... Please do not move the robot."<<std::endl;
        }
        delta_th = 0.0;
        gyro_drift_velocity = 0.0;
        gyro_drift_acceleration = 0.0;

        if (count > garbage_length){
            if(count == garbage_length + 1){
                starting_time = ros::Time::now();
            }
            sum_th += sync_yaw_rate;
            //sum_t += t;
            //sum_t_t += t * t;
            //sum_t_th += t * sync_yaw_rate;
            //sum_dth += (sync_yaw_rate1 - sync_yaw_rate) / dt;
        }

        if(count > sample_length){
            int n = count - garbage_length;
            gyro_drift_acceleration = 0;
            gyro_drift_velocity = sum_th / n;
            sum_th = 0.0;
            sum_t = 0.0;
            sum_t_t = 0.0;
            sum_t_th = 0.0;
            count = 0;
            calibrate = false;

            std::cout<<"Calibration finished! "<<gyro_drift_velocity<<"\t"<<gyro_drift_acceleration<<std::endl;
        }
        count++;
    }



    x += delta_x;
    y += delta_y;
    th += delta_th;
    t += dt;

    if(print_data){
        std::cout<<th<<"\t"<<sync_yaw_rate<<"\t"<<t<<std::endl;
    }

    if(acquire_data){
        data.open("data.txt",std::ios_base::app);
        data<<sync_yaw_rate<<"\t"<<(t-dt)<<"\n";
        data.close();
    }



    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";


    odom_trans.transform.rotation = odom_quat;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;


    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    std_msgs::Float64 heading;
    heading.data = th;
    yaw_pub.publish(heading);

    last_time = current_time;
    sync_yaw_rate = yaw_rate;
    sync_speed = speed;
    r.sleep();
  }
}
