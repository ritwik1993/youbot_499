#include "youbot_499/arm_interface_youbot.h"
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <iostream>
#include <sstream>

////////////////////////////////////////////////////////////////////////////////
//  Construction / Destruction
////////////////////////////////////////////////////////////////////////////////

cArmInterfaceYoubot::cArmInterfaceYoubot(const tf::Transform& g_arm0_to_base_link, ros::NodeHandle& nh)
	: cArmInterface(g_arm0_to_base_link)
{
	mArmJointsPub = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);
	mGripperPub = nh.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 1);
}

////////////////////////////////////////////////////////////////////////////////
//  Interface Functions
////////////////////////////////////////////////////////////////////////////////

void cArmInterfaceYoubot::PublishJointValues(const std::vector<double>& values)
{
	brics_actuator::JointPositions pos;
	for( std::size_t i = 0; i < values.size(); ++i )
	{
		brics_actuator::JointValue val;
		
		std::ostringstream jointName;
		jointName << "arm_joint_" << i+1;
		val.joint_uri = jointName.str();
		val.unit = "rad";
		val.value = values[i];

		pos.positions.push_back(val);
	}

	mArmJointsPub.publish(pos);
}

////////////////////////////////////////////////////////////////////////////////
//  Helper Functions
////////////////////////////////////////////////////////////////////////////////

void cArmInterfaceYoubot::PublishGripperValues(double width)
{
	brics_actuator::JointPositions pos;

	brics_actuator::JointValue finger;
	finger.joint_uri = "gripper_finger_joint_l";
	finger.unit = "m";
	finger.value = width;
	pos.positions.push_back(finger);

	finger.joint_uri = "gripper_finger_joint_r";
	finger.unit = "m";
	finger.value = width;
	pos.positions.push_back(finger);

	mGripperPub.publish(pos);
}
