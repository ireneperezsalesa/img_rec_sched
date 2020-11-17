// RECONSTRUCTION - GREY BACKGROUND, USING PREVIOUS IMAGE TO ADD NEW EVENTS - with scheduling

#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <img_rec_sched/Trigger.h>


cv_bridge::CvImage cv_image;
cv_bridge::CvImage last;

int counter = 1;
bool got_first_image = false; 

int next_ack;

void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
	if (got_first_image == false)
	{	
		cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC1);
        	cv_image.image = cv::Scalar(128);
		cv_image.encoding="mono8";
		got_first_image = true;
	}
	else {
		cv_image.image = last.image;
	}

	for (int i = 0; i < msg->events.size(); ++i)
	{
        	const int x = msg->events[i].x;
        	const int y = msg->events[i].y;

        	int delta = 30;
            	if (msg->events[i].polarity == true && cv_image.image.at<uint8_t>(cv::Point(x, y)) < (uint8_t)(255-delta))  
		{
			cv_image.image.at<uint8_t>(cv::Point(x, y))+=(uint8_t)delta; // adjust value here
		}
		else if (msg->events[i].polarity == true) 
		{
			cv_image.image.at<uint8_t>(cv::Point(x, y))=(uint8_t)255;
		}		
		else if (msg->events[i].polarity == false && cv_image.image.at<uint8_t>(cv::Point(x, y)) > (uint8_t)delta)
		{
			cv_image.image.at<uint8_t>(cv::Point(x, y))-=(uint8_t)delta; 
		}
		else 
		{
			cv_image.image.at<uint8_t>(cv::Point(x, y))=(uint8_t)0;
		}
		
	}

	last.image = cv_image.image;

}


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "reconstructor");

	ros::NodeHandle n; 

	image_transport::ImageTransport it(n);

	// service client for scheduling
	ros::ServiceClient client = n.serviceClient<img_rec_sched::Trigger>("trigger_camera"); 
	img_rec_sched::Trigger srv;

	// subscribe to /dvs/events 
	ros::Subscriber events_sub = n.subscribe("/dvs/events", 1, eventsCallback);
	std::cout << "Subscribed to /dvs/events." << std::endl;

	// publish new image
	image_transport::Publisher img_pub = it.advertise("reconstructed", 1);
	std::cout << "Publishing /reconstructed." << std::endl;

	ros::Time last = ros::Time::now();

	srv.request.ack = 0;

	next_ack = 0;

	while(ros::ok())
	{
		if (next_ack == 1)
		{
			srv.request.ack = 1;
		}
		else 
		{
			srv.request.ack = 0;
		}

		// callback				
		ros::spinOnce();

		if (client.call(srv))
		{
			if (srv.response.trigger == 1)
			{ 
				// publish reconstructed image only when scheduler indicates
				img_pub.publish(cv_image.toImageMsg());	
				ros::Time current = ros::Time::now();
				ros::Duration time = current - last;
				std::cout << "Interval: " << time.toSec() << std::endl;
				last = current;
				next_ack = 1;
			}
			else 
			{
				next_ack = 0;
			}
			//std::cout << "Service called" << std::endl;
		}
		else
		{
			ROS_ERROR("Failed to call service trigger_camera");
		}
				
	}

}
