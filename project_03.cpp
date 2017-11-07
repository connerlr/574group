//////testing push
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <cmath>

/******************************************************************************
 *3456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789
 * Class 'project_03ls
' for the queueing theory computation.
 *
 * Author/copyright: Conner Roberts
 * 
 * Date last modified: 11 October 2017
**/

class project_03
{
	public:

	//
	double wallDis, errorMargin, derivative, Speed, pConst, dConst,
		   direction, angleMin, toWall, brownie;
	ros::Publisher pub;
	
	//constructor
	project_03(ros::Publisher paramPub)
	{
		pub = paramPub;
		wallDis = 0.6;//desired distance to stay away from the wall
		Speed = 0.4;//full speed when nothing is in front
		pConst = 1;
		dConst = 1;
		errorMargin = 0;
		derivative = 0;
		direction = -1; //the right wall
		brownie = 3.14159265359; //pie is the obvious inferior dessert	
	}
	
	~project_03()
	{
	}

	/**
	* Function 'messagePub()'
	* This function publishes the desired speeds to the robot
	**/
	void messagePub()
	{
		ROS_INFO_STREAM("Enters messagepub");
		geometry_msgs::Twist msg;
		
		//formula for the PD controller	
		//ROS_INFO_STREAM(errorMargin);
		//ROS_INFO_STREAM(derivative);
		//ROS_INFO_STREAM(angleMin);
		msg.angular.z = direction*(pConst*errorMargin + dConst*derivative) + 
								  (angleMin + brownie/2);
		ROS_INFO_STREAM(msg.angular.z);
		//msg.angular.z = 0.0;
		//msg.linear.x = 0.1;
		if(toWall < wallDis) //if you are already too close
		{
			msg.linear.x = 0;
		}
		else if(toWall < wallDis * 2)//if you see the wall getting close
		{
			msg.linear.x = 0.5*Speed;
		}
		//if you see the wall getting close at the side
		else if(fabs(angleMin)>1.75)
		{
			msg.linear.x = 0.4*Speed;
		}
		else // if you are all clear
		{
			msg.linear.x = Speed;
		}
		pub.publish(msg);
	}
	/**
	* Function 'getLaser()'
	* This function receives the messages from the robot and records them 
	* while storing the data.
	*
	* Parameters:
	* msg - msg that the robot sends and our subscriber receives.
	**/

	bool thisisNan(double numOrNotNum)
	{
		bool thing;
		if(numOrNotNum == numOrNotNum)
			thing = false;
		if(numOrNotNum != numOrNotNum)
			thing = true;
		return thing;
	}
	
	void getLaser(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		ROS_INFO_STREAM("Enters laserscan");
		int size = msg->ranges.size();
		//variable with lowest index in array
		int indexLow = size*(direction + 1) / 4;
		//variable with highest index in array
		int indexHigh = size*(direction + 3) /4;
		//indexes through array to find the lowest index


		for(int i = 0; i < size; i++);
		int nanCount = 0;
		for(int i = indexLow; i < indexHigh; i++)
		{
			if(thisisNan(msg->ranges[i]) == false)
			{
				if(nanCount == 0)
				{
					indexLow = i;
				}
				ROS_INFO_STREAM(msg->ranges[i]);
				if((msg->ranges[i] < msg->ranges[indexLow]) && (msg->ranges[i] > 0.0))
				{	
					indexLow = i;
				}
				nanCount++;
			}
		}
		
		ROS_INFO_STREAM(indexLow);
		//calculates the minimum angle
		angleMin = (indexLow - size / 2) * msg->angle_increment;
		double distanceMin;
		distanceMin = msg->ranges[indexLow];
		toWall = msg->ranges[size/2];
		derivative = (distanceMin - wallDis) - errorMargin;
		errorMargin = distanceMin - wallDis;
		messagePub();
	}
};

int main(int argc, char **argv)
{
	ROS_INFO_STREAM("Enters Main");
	ros::init(argc, argv, "project_03");
	ros::NodeHandle nh;
	ros::Publisher pubMes = nh.advertise<geometry_msgs::Twist>
		("/mobile_base/commands/velocity", 1000);//create the publisher
	//create our custom object for the pd controller
	project_03 *cntrl = new project_03(pubMes);
	//create the subscriber
	ros::Subscriber sub = nh.subscribe("/scan", 1000, &project_03::getLaser, cntrl);
	ROS_INFO_STREAM("Subscriber created");
	ros::spin();
	return 0;
	

}
