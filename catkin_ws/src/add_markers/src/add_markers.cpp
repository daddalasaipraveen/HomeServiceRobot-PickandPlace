#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(1);

	// to confirm the topic to be subscribed
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// to check wheather the robot is at pickup/dropoff
	bool is_robot_at_pickup_zone = false;
	bool is_robot_at_dropoff_zone = false;

	// Set our initial shape type to be a cube
	visualization_msgs::Marker marker;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER, but we can choose anyone
	marker.type = visualization_msgs::Marker::CUBE;

	// Set the frame ID and timestamp
	marker.header.frame_id = "map";

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	// here namespace can choose according to the place where we work on like park, home, etc
	marker.ns = "add_markers";
	marker.id = 0;


	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	// Pose
	marker.pose.position.x = 2;
	marker.pose.position.y = 2;
	marker.pose.position.z = 0;
	// Orientation
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	while (ros::ok())
	{
		
		// Set timestamp
		marker.header.stamp = ros::Time::now();

		// Set marker action with the options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;

		
		// marker.lifetime = ros::Duration();

		// To Publish the Marker at PICKUP location
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN("Please create a subscriber to the marker");
			sleep(1);
			ROS_INFO("Print after sleep 2");
		}
		marker_pub.publish(marker);
		
		ros::spinOnce();
		r.sleep();

		//ros::Duration(5).sleep()
		n.getParam("is_robot_at_pickup_zone", is_robot_at_pickup_zone);
		if (is_robot_at_pickup_zone)
			break;
	}



	// To Delete the Marker at DROPOFF location
	ROS_INFO("Deleting the marker(under process)");
	marker.action = visualization_msgs::Marker::DELETE;
	marker_pub.publish(marker);
	// ros::Duration(5).sleep()

	// here the code will block until the robot has reached drop-off zone
	while (true)   
	{
		n.getParam("is_robot_at_dropoff_zone", is_robot_at_dropoff_zone);
		if (is_robot_at_dropoff_zone)
			break;
	}


	// The following code explains to further reach Goal by adding marker and asking the robot to reach goal
	// To add the Marker at DROPOFF location (follow the above steps)
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	// Pose
	marker.pose.position.x = -2;
	marker.pose.position.y = 3;
	marker.pose.position.z = 0.0;
	// Orientation
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 1.0;
	marker.pose.orientation.z = 1.0;
	marker.pose.orientation.w = 1.0;

	ROS_INFO("At this point, the Marker is added at DROPOFF location");

	marker.lifetime = ros::Duration();	

	while (ros::ok())
	{
		
		// Set timestamp
		marker.header.stamp = ros::Time::now();

		// Set marker action with the options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;

		// marker.lifetime = ros::Duration();

		// Finally to Publish the Marker at DROPOFF location
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN("Please create a subscriber to the marker");
			sleep(1);
			ROS_INFO("Print after sleep 2");
		}
		marker_pub.publish(marker);
		
		ros::spinOnce();
		r.sleep();
	}  
}
