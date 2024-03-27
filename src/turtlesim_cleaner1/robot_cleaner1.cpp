#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include<math.h>

ros::Publisher velocity_publisher;
using namespace std;
void move(double speed, double distance, double speedAngle, double angle, bool isForward, bool isLeft);

int main(int argc, char **argv){
	ros::init(argc, argv, "robot_cleaner");
	ros::NodeHandle n;

	double speed;
	double distance;
	double speedAngle;
	double angle;
	bool isForward;
	bool isLeft;

	velocity_publisher =
n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	cout<<"enter speed:";
	cin>>speed;
	cout<<"enter distance: ";
	cin>>distance;
	cout<<"enter speed angle:";
	cin>>speedAngle;
	cout<<"isForward?: ";
	cin>>isForward;
	cout<<"enter angle: ";
	cin>>angle;
	cout<<"isLeft?: ";
	cin>>isLeft;
	move(speed, distance, speedAngle, angle, isForward, isLeft);

}

void move(double speed, double distance, double speedAngle, double angle, bool isForward, bool isLeft){
	geometry_msgs::Twist vel_msg;
	if (isForward)
		vel_msg.linear.x=abs(speed);
	else
		vel_msg.linear.x=-abs(speed);
	vel_msg.linear.y=0;
	vel_msg.linear.z=0;

	vel_msg.angular.x=0;
	vel_msg.angular.y=0;

	if (isLeft)
		vel_msg.angular.z=-abs(speedAngle);
	else
		vel_msg.angular.z=abs(speedAngle);

	
	double distance_init=distance;
	double increment=0.1;
	double compteur=0;

	double t0=0;
	double t1=0;
	double t2=0;
	double t3=0;
	double t4=0;
	double t5=0;
	double current_distance=0;
	double current_angle=0;
	double current_distance_2=0;
	


	do{

	t0 = ros::Time::now().toSec();
	current_distance=0;
	ros::Rate loop_rate(500);
	do{
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		current_distance = speed*(t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_distance < distance+increment*compteur);
	
	vel_msg.linear.x=0;
	velocity_publisher.publish(vel_msg);
	
	t2 = ros::Time::now().toSec();
	current_angle=0;
	do{
		vel_msg.angular.z=50*M_PI/180;
		velocity_publisher.publish(vel_msg);
		t3 = ros::Time::now().toSec();
		current_angle = vel_msg.angular.z*(t3-t2);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle < M_PI/2);
	vel_msg.angular.z=0;
	velocity_publisher.publish(vel_msg);
	
	t4 = ros::Time::now().toSec();
	current_distance_2=0;
	do{
		vel_msg.linear.x=2;
		velocity_publisher.publish(vel_msg);
		t5 = ros::Time::now().toSec();
		current_distance_2 = vel_msg.linear.x*(t5-t4);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_distance_2 < distance+increment*compteur);
	vel_msg.linear.x=0;
	velocity_publisher.publish(vel_msg);
	
	t0 = ros::Time::now().toSec();
	current_distance=0;
	do{
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		current_distance = speed*(t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_distance < distance+increment*compteur);
	
	vel_msg.linear.x=0;
	velocity_publisher.publish(vel_msg);
	
	t2 = ros::Time::now().toSec();
	current_angle=0;
	do{
		vel_msg.angular.z=50*M_PI/180;
		velocity_publisher.publish(vel_msg);
		t3 = ros::Time::now().toSec();
		current_angle = vel_msg.angular.z*(t3-t2);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle < M_PI/2);
	vel_msg.angular.z=0;
	velocity_publisher.publish(vel_msg);
	
	t4 = ros::Time::now().toSec();
	current_distance_2=0;
	do{
		vel_msg.linear.x=2;
		velocity_publisher.publish(vel_msg);
		t5 = ros::Time::now().toSec();
		current_distance_2 = vel_msg.linear.x*(t5-t4);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_distance_2 < distance+2*increment*compteur);
	vel_msg.linear.x=0;
	velocity_publisher.publish(vel_msg);
	compteur=compteur+1;
	}while(1);
}
