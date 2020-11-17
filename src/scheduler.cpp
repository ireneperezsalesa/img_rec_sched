// to run: roslaunch camera_sched params.launch first and then rosrun camera_sched scheduler

// read schedule parameters from a yaml file, trigger camera node

#include "ros/ros.h"
#include "img_rec_sched/Trigger.h"
#include <iostream>

float t1;
float t2;
float t3;

int flag1;
int flag2;
int flag3;

bool t1done;
bool t2done;
bool t3done;

bool set_timer1;
bool set_timer2;
bool set_timer3;

ros::Time init;

ros::Timer timer1;
ros::Timer timer2;
ros::Timer timer3;

bool wait_trigger(img_rec_sched::Trigger::Request  &req,
         img_rec_sched::Trigger::Response &res) 
{
	
	if (req.ack == 1)
	{
		flag1 = 0;
		flag2 = 0;
		flag3 = 0;
		res.trigger = 0;
	}
	
	else if ((flag1 == 1) or (flag2 == 1) or (flag3 == 1))
	{
		res.trigger = 1;
	}

	else
	{
		res.trigger = 0;
	}

	return true;
} // returns true if service was successful

void timerCallback1(const ros::TimerEvent&) 
{
	if (t1done == false)
	{
		flag1 = 1;
		t1done = true;
		t2done = false;
		ros::Duration time = ros::Time::now() - init;
		set_timer2 = true;
		std::cout << "Timer 1. Time: " << time.toSec() << std::endl;

	}
}

void timerCallback2(const ros::TimerEvent&) 
{
	if (t2done == false)
	{
		flag2 = 1;
		t2done = true;
		t3done = false;
		set_timer3 = true;
		ros::Duration time = ros::Time::now() - init;
		std::cout << "Timer 2. Time: " << time.toSec() << std::endl;
	}
}

void timerCallback3(const ros::TimerEvent&) 
{
	if (t3done == false)
	{
		flag3 = 1;
		t3done = true;
		t1done = false;
		set_timer1 = true;
		ros::Duration time = ros::Time::now() - init;
		std::cout << "Timer 3. Time: " << time.toSec() << std::endl;
	}
}


int main(int argc, char **argv) 
{
	// Initialization
	ros::init(argc, argv, "scheduler");

	// ROS node handle
	ros::NodeHandle n;

	std::cout << "Running scheduler." << std::endl;

	// get parameters for scheduling
	n.getParam("/time1", t1);
	n.getParam("/time2", t2);
	n.getParam("/time3", t3);

	std::cout << "Parameters: \n" << "t1 = " << t1 << "; t2 = " << t2 << "; t3 = " << t3  << std::endl;

	// advertise service
	ros::ServiceServer service = n.advertiseService("trigger_camera", wait_trigger);

	std::cout << "Waiting for client." << std::endl;

	t1done = false;
	t2done = false;

	flag1 = 0;
	flag2 = 0;
	flag3 = 0;

	set_timer1 = false;
	set_timer2 = false;
	set_timer3 = false;

	init = ros::Time::now();

	int first = 0;

	while(ros::ok())
	{

		if (first == 0)
		{
			timer1 = n.createTimer(ros::Duration(t1), timerCallback1); 
			first++;
		}

		if (set_timer2 == true)
		{
			timer2 = n.createTimer(ros::Duration(t2), timerCallback2); 
			set_timer2 = false;
		}

		if (set_timer3 == true)
		{
			timer3 = n.createTimer(ros::Duration(t3), timerCallback3); 
			set_timer3 = false;
		}
		
		if (set_timer1 == true)
		{
			timer1 = n.createTimer(ros::Duration(t1), timerCallback1); 
			set_timer1 = false;
		}

		ros::spinOnce();

	}
	
	return 0;
}
