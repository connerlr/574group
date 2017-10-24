#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

/******************************************************************************
 *3456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789
 * Class 'pdc' for the queueing theory computation.
 *
 * Author/copyright: Conner Roberts
 * 
 * Date last modified: 11 October 2017
**/

class pdc
{
	public:

	//
	double wallDis, errorMargin, derivative, Speed, pConst, dConst,
		   direction, angleMin, toWall, brownie;
	ros::Publisher pub;
	
	//constructor
	pdc(ros::Publisher paramPub)
	{
		pub = paramPub;
		wallDis = 0.6;//desired distance to stay away from the wall
		Speed = 0.7;//full speed when nothing is in front
		pConst = 5;
		dConst = 5;
		errorMargin = 0;
		derivative = 0;
		direction = -1; //the right wall
		brownie = 3.14159265359; //pie is the obvious inferior dessert	
	}
	
	~pdc()
	{
	}

	/**
	* Function 'messagePub()'
	* This function publishes the desired speeds to the robot
	**/
	void messagePub()
	{
		geometry_msgs::Twist msg;
		
		//formula for the PD controller
		msg.angular.z = direction*(pConst*errorMargin + dConst*derivative) + 
								  (angleMin + brownie/2);
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
	void getLaser(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		int size = msg->ranges.size();
		//variable with lowest index in array
		int indexLow = size*(direction + 1) / 4;
		//variable with highest index in array
		int indexHigh = size*(direction + 3) /4;
		//indexes through array to find the lowest index

		for(int i = indexLow; i < indexHigh; i++)
		{
			if(msg->ranges[i] < msg->ranges[indexLow] && msg->ranges[i] > 0.0)
			{
				indexLow = i;
			}
		}
		//calculates the minimum angle
		angleMin = (indexLow - size / 2) * msg->angle_increment;
		double distanceMin = msg->ranges[indexLow];
		toWall = msg->ranges[size/2];
		derivative = (distanceMin - wallDis) - errorMargin;
		errorMargin = distanceMin - wallDis;
		messagePub();
	}
};

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "pdc");
	ros::NodeHandle nh;
	ros::Publisher pubMes = nh.advertise<geometry_msgs::Twist>
		("cmd_vel", 1000);//create the publisher
	//create our custom object for the pd controller
	pdc *cntrl = new pdc(pubMes);
	//create the subscriber
	ros::Subscriber sub = nh.subscribe("base_scan", 10, &pdc::getLaser, cntrl);
	ros::spin();
	return 0;
	

}
