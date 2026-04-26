/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Modified on: 11/12/22
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi>
 */

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_intro");
    ros::NodeHandle node("~");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // This is the geometry message that you will populate with the lumi_ee to base_link transformation
    geometry_msgs::TransformStamped ee_to_base_pose;
    
    /* TODO: Create a publisher that publishes the transformation to the topic tf_pose */
	// ros::Publisher name_of_publisher = ...

    /* TODO: Set the ros loop rate to 25 hz */
	// ros::Rate name_of_looprate...
	
    while (ros::ok())
    {
     try {
		/* TODO: Read the transformation from lumi_ee to base_link into the variable ee_to_base_pose.
			(Hint: Have look at the example here http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29.
			Make sure you have the order of the arguments (source_frame, target_frame) correctly) */
	    // ee_to_base_pose = ...
    } catch (tf2::TransformException &ex) {
        ROS_ERROR_STREAM("Cannot find transformation from lumi_ee to base_link. What: " << ex.what());
        return -1;
    }
		/* TODO: Publish the recently read transformation. Make sure you have name_of_looprate.sleep() after rosSpin*/
        ros::spinOnce();
    }
    return 0;
}
