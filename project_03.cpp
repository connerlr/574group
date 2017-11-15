//////testing push
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <time.h>
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
		   direction, angleMin, toWall, brownie, angle, x_curr, y_curr, theta_curr,
		   turn_angle, move_distance;
	ros::Publisher pub;
	
	//constructor
	project_03(ros::Publisher paramPub)
	{
		move_distance = 0.4;
		turn_angle = 1.13446; //90 degrees in radians
		x_curr = 0;
		y_curr = 0;
		theta_curr = 0;
		pub = paramPub;
		wallDis = 0.5;//desired distance to stay away from the wall
		Speed = 0.4;//full speed when nothing is in front
		pConst = 1;
		dConst = 1;
        angle = 1;
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
		geometry_msgs::Twist msg;
		
		//formula for the PD controller	
		//ROS_INFO_STREAM(errorMargin);
		//ROS_INFO_STREAM(derivative);
		//ROS_INFO_STREAM(toWall);
		msg.angular.z = (direction*(pConst*errorMargin + dConst*derivative) + 
								  angle*(angleMin + brownie/2));
	    
		//ROS_INFO_STREAM(msg.angular.z);
		

		if(toWall < wallDis) //if you are already too close
		{
			msg.linear.x = 0.1;
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

	void turnNinety()
	{
		geometry_msgs::Twist msg;
		nav_msgs::OdometryConstPtr odomMsg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
		double theta = tf::getYaw(odomMsg->pose.pose.orientation);
		//Set the movement command rotation speed
		msg.angular.z = 0.6;
		// Current angle
		double last_angle = theta;
		double angle = 0;

		while ((angle < turn_angle) && ros::ok()) 
		{
		    nav_msgs::OdometryConstPtr odomMsg2 = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
			theta = tf::getYaw(odomMsg2->pose.pose.orientation);

			//Publish the Twist message and sleep for a cycle
		    pub.publish(msg);

		    // Compute the amount of rotation since the last loop
		    angle += angles::normalize_angle(theta - last_angle);
            
		    last_angle = theta;

		}

		//Robot stops turning
		msg.angular.z = 0;
		pub.publish(msg);
		driveForward();
	}

	void driveForward()
	{
		geometry_msgs::Twist msg;
		nav_msgs::OdometryConstPtr odomMsg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
		double x = -odomMsg->pose.pose.position.y;
		double y = odomMsg->pose.pose.position.x;
		//Set the movement command rotation speed
		msg.linear.x = 0.4;
		// Current angle
		double last_x = x;
		double last_y = y;
		double d = 0;

		while ((d < move_distance) && ros::ok()) 
		{
		    nav_msgs::OdometryConstPtr odomMsg2 = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
			x = -odomMsg2->pose.pose.position.y;
			y = odomMsg2->pose.pose.position.x;

			//Publish the Twist message and sleep for a cycle
		    pub.publish(msg);

		    d = sqrt(pow((x - last_x), 2) + pow((y - last_y), 2));
		}

		//Robot stops turning
		msg.linear.x = 0;
		pub.publish(msg);
		turnNinetyRight();
	}

	void turnNinetyRight()
	{
		geometry_msgs::Twist msg;
		nav_msgs::OdometryConstPtr odomMsg = 
			ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
		double theta = tf::getYaw(odomMsg->pose.pose.orientation);
		//Set the movement command rotation speed
		msg.angular.z = -0.6;
		// Current angle
		double last_angle = theta;
		double angle = 0;

		while ((angle > -turn_angle) && ros::ok()) 
		{
		    nav_msgs::OdometryConstPtr odomMsg2 = 
				ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
			theta = tf::getYaw(odomMsg2->pose.pose.orientation);

			//Publish the Twist message and sleep for a cycle
		    pub.publish(msg);

		    // Compute the amount of rotation since the last loop
		    angle += angles::normalize_angle(theta - last_angle);
            
		    last_angle = theta;

		}

		//Robot stops turning
		msg.angular.z = 0;
		pub.publish(msg);
	}
	
	void getLaser(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		int size = msg->ranges.size();
		//variable with lowest index in array
		int indexLow = size*(direction + 1) / 4;
		//variable with highest index in array
		int indexHigh = size*(direction + 3) /4;
		//indexes through array to find the lowest index

		int nanCount = 0;
		for(int i = indexLow; i < indexHigh; i++)
		{
			if(thisisNan(msg->ranges[i]) == false)
			{
				if(nanCount == 0)
				{
					indexLow = i;
				}
				if((msg->ranges[i] < msg->ranges[indexLow]) && (msg->ranges[i] > 0.0))
				{	
					indexLow = i;
				}
				nanCount++;
			}
		}
		
		//calculates the minimum angle
		angleMin = ((indexLow - size / 2) * msg->angle_increment);
		double distanceMin;
		if(thisisNan(msg->ranges[indexLow]) == false)
		{
			distanceMin = msg->ranges[indexLow];
		}
		else
		{
			turnNinety();	
		}
		if(thisisNan(msg->ranges[size/2]) == false)
		{
			toWall = msg->ranges[size/2];
		}
		derivative = ((distanceMin - wallDis) - errorMargin);
		errorMargin = (distanceMin - wallDis);
		messagePub();
	}


	void getOdom(const nav_msgs::Odometry::ConstPtr& msg)
	{
		x_curr = msg->pose.pose.position.x;
   		y_curr = msg->pose.pose.position.y;
   		// quaternion to RPY conversion
    	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
					 msg->pose.pose.orientation.z, 
    	msg->pose.pose.orientation.w);
    	tf::Matrix3x3 m(q);
    	double roll, pitch, yaw;
    	m.getRPY(roll, pitch, yaw);
		// angular position
		theta_curr = yaw;
		ROS_INFO("THETA: %f", theta_curr); 
	}
};

int main(int argc, char **argv)
{
	ROS_INFO_STREAM("Enters Main");
	ros::init(argc, argv, "project_03");
	ros::NodeHandle nh;
	ros::Publisher pubMes = nh.advertise<geometry_msgs::Twist>
		("/mobile_base/commands/velocity", 100);//create the publisher
	//create our custom object for the pd controller
	project_03 *cntrl = new project_03(pubMes);
	//create the subscriber	
	ros::Subscriber odomSub = nh.subscribe("/odom", 10, &project_03::getOdom, cntrl);
	ros::Subscriber sub = nh.subscribe("/scan", 10, &project_03::getLaser, cntrl);

	ROS_INFO_STREAM("Subscriber created");
	ros::spin();
	return 0;
	

}
