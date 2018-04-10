#include <robot_interface/robot_interface_node.h>
#include <iterator>


CanInterfaceNode::CanInterfaceNode(ros::NodeHandle *n){
	this->nh = *n;

	this->can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 1000);
	this->test_pub = nh.advertise<sender::test2>("receiver/test", 1000);

	this->STM_coder_pub = nh.advertise<robot_interface::WheelsDistance>("/STM/GetCoder", 10);
	this->STM_pos_pub = nh.advertise<robot_interface::Point>("/STM/Position", 10);
	this->STM_pwm_pub = nh.advertise<robot_interface::PWMs>("/STM/GetPWM", 10);
	this->STM_speed_pub = nh.advertise<robot_interface::CurrSpeed>("/STM/GetSpeed", 10);

	this->robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_watcher", 1000, &CanInterfaceNode::updateRobotStatus, this);
	this->test_sub = nh.subscribe("/ros_can/interface/test", 1000, &CanInterfaceNode::test, this);
	this->can_sub = nh.subscribe("received_messages", 1000, &CanInterfaceNode::canMsgProcess, this);

	this->STMSetMode_sub = nh.subscribe("/STM/SetMode",10, &CanInterfaceNode::STMSetMode, this);
	this->STMSpeed_sub = nh.subscribe("/STM/Speed",10, &CanInterfaceNode::STMSpeed, this);
	this->STMAsserManagement_sub = nh.subscribe("/STM/AsserManagement",10, &CanInterfaceNode::STMAsserManagement, this);
	this->STMGoToAngle_sub = nh.subscribe("/STM/GoToAngle",10, &CanInterfaceNode::STMGoToAngle, this);
	this->STMGoTo_sub = nh.subscribe("/STM/GoTo",10, &CanInterfaceNode::STMGoTo, this);
	this->STMRotation_sub = nh.subscribe("/STM/Rotation",10, &CanInterfaceNode::STMRotation, this);
	this->STMRotationNoModulo_sub = nh.subscribe("/STM/RotationNoModulo",10, &CanInterfaceNode::STMRotationNoModulo, this);
	this->STMLeftPID_sub = nh.subscribe("/STM/LeftPID",10, &CanInterfaceNode::STMLeftPID, this);
	this->STMRightPID_sub = nh.subscribe("/STM/RightPID",10, &CanInterfaceNode::STMRightPID, this);
	this->STMAllPID_sub = nh.subscribe("/STM/AllPID",10, &CanInterfaceNode::STMAllPID, this);
	this->STMPWM_sub = nh.subscribe("/STM/PWM",10, &CanInterfaceNode::STMPWM, this);
	this->STMSetPose_sub = nh.subscribe("/STM/SetPose",10, &CanInterfaceNode::STMSetPose, this);
	this->STMSetParam_sub = nh.subscribe("/STM/SetParam",10, &CanInterfaceNode::STMSetParam, this);

	service_ready("ros_can", "interface", 1 );

}

CanInterfaceNode::~CanInterfaceNode(){

}

void CanInterfaceNode::updateRobotStatus(const robot_watcher::RobotStatus::ConstPtr& msg){
	this->robot_watcher = msg->robot_watcher;
	// ROS_INFO("callback robot_watcher: %d", this->robot_watcher);
}

void CanInterfaceNode::test(const sender::test2::ConstPtr& msg){
	// ROS_INFO("callback mode: %d", msg->mode);
	// for (int i = 0; i < 7; ++i)
	// {
	// 	ROS_INFO("callback data %d: %d",i, msg->data[i]);
	// }
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.id = 1;
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;
	fr.dlc = 8;
	fr.data[0] = msg->mode;
	std::copy(std::begin(msg->data), std::end(msg->data), std::next(std::begin(fr.data),1));

}

void CanInterfaceNode::canMsgProcess(const can_msgs::Frame::ConstPtr& msg){

	switch (msg->data[0]) {
		case WHOAMI:{

			uint boardID = msg->id;
			uint8_t status = msg->data[1];
			break;
			}
		case GET_CODER:{
			robot_interface::WheelsDistance msg_out;
			msg_out.right_wheel_dist = msg->data[2] | msg->data[1] << 8; //mm
			msg_out.left_wheel_dist = msg->data[4] | msg->data[3] << 8; //mm

			this->STM_coder_pub.publish(msg_out);
			break;
		}
		case CURRENT_POS:{
		robot_interface::Point msg_out;

			msg_out.pos_x = msg->data[2] | msg->data[1] << 8; //mm
			msg_out.pos_y = msg->data[4] | msg->data[3] << 8; //mm
			msg_out.angle = msg->data[6] | msg->data[5] << 8; //mRad

			this->STM_pos_pub.publish(msg_out);
			break;
		}
		case CURRENT_PWM:{
			robot_interface::PWMs msg_out;

			msg_out.left_pwm = msg->data[2] | msg->data[1] << 8;
			msg_out.right_pwm = msg->data[4] | msg->data[3] << 8;

			this->STM_pwm_pub.publish(msg_out);
			break;
		}
		case CURRENT_SPD:{
			robot_interface::CurrSpeed msg_out;

			msg_out.linear_speed = msg->data[2] | msg->data[1] << 8; //mm/s
			msg_out.left_speed = msg->data[4] | msg->data[3] << 8; //mm/s
			msg_out.right_speed = msg->data[6] | msg->data[5] << 8; //mm/s

			this->STM_speed_pub.publish(msg_out);
			break;
		}
	}

}


void CanInterfaceNode::STMSetMode(const robot_interface::Status::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 2;
	fr.id = STM_ID;
	fr.data[0] = SET_MODE;
	fr.data[1] = msg->value;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMSpeed(const robot_interface::Speed::ConstPtr& msg){

	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 7;
	fr.id = STM_ID;
	fr.data[0] = SPEED;
	fr.data[1] = msg->linear_speed >> 8;
	fr.data[2] = msg->linear_speed & 0x00FF;
	fr.data[3] = msg->angular_speed >> 8;
	fr.data[4] = msg->angular_speed & 0x00FF;
	fr.data[5] = msg->duration >> 8;
	fr.data[6] = msg->duration & 0x00FF;

	can_pub.publish(fr);

}

void CanInterfaceNode::STMAsserManagement(const robot_interface::Status::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 2;
	fr.id = STM_ID;
	fr.data[0] = MANAGEMENT;
	fr.data[1] = msg->value;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMGoToAngle(const robot_interface::Point::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 8;
	fr.id = STM_ID;
	fr.data[0] = GOTOA;
	fr.data[1] = msg->pos_x >> 8;
	fr.data[2] = msg->pos_x & 0x00FF;
	fr.data[3] = msg->pos_y >> 8;
	fr.data[4] = msg->pos_y & 0x00FF;
	fr.data[5] = msg->angle >> 8;
	fr.data[6] = msg->angle & 0x00FF;
	fr.data[7] = msg->direction;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMGoTo(const robot_interface::Point::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 6;
	fr.id = STM_ID;
	fr.data[0] = GOTO;
	fr.data[1] = msg->pos_x >> 8;
	fr.data[2] = msg->pos_x & 0x00FF;
	fr.data[3] = msg->pos_y >> 8;
	fr.data[4] = msg->pos_y & 0x00FF;
	fr.data[5] = msg->direction;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMRotation(const robot_interface::Point::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 3;
	fr.id = STM_ID;
	fr.data[0] = ROT;
	fr.data[1] = msg->angle >> 8;
	fr.data[2] = msg->angle & 0x00FF;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMRotationNoModulo(const robot_interface::Point::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 3;
	fr.id = STM_ID;
	fr.data[0] = ROT;
	fr.data[1] = msg->angle >> 8;
	fr.data[2] = msg->angle & 0x00FF;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMLeftPID(const robot_interface::PID::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 7;
	fr.id = STM_ID;
	fr.data[0] = PIDLEFT;
	fr.data[1] = msg->P >> 8;
	fr.data[2] = msg->P & 0x00FF;
	fr.data[3] = msg->I >> 8;
	fr.data[4] = msg->I & 0x00FF;
	fr.data[5] = msg->D >> 8;
	fr.data[6] = msg->D & 0x00FF;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMRightPID(const robot_interface::PID::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 7;
	fr.id = STM_ID;
	fr.data[0] = PIDRIGHT;
	fr.data[1] = msg->P >> 8;
	fr.data[2] = msg->P & 0x00FF;
	fr.data[3] = msg->I >> 8;
	fr.data[4] = msg->I & 0x00FF;
	fr.data[5] = msg->D >> 8;
	fr.data[6] = msg->D & 0x00FF;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMAllPID(const robot_interface::PID::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 7;
	fr.id = STM_ID;
	fr.data[0] = PIDALL;
	fr.data[1] = msg->P >> 8;
	fr.data[2] = msg->P & 0x00FF;
	fr.data[3] = msg->I >> 8;
	fr.data[4] = msg->I & 0x00FF;
	fr.data[5] = msg->D >> 8;
	fr.data[6] = msg->D & 0x00FF;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMPWM(const robot_interface::PWMs::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 5;
	fr.id = STM_ID;
	fr.data[0] = PWM;
	fr.data[1] = msg->left_pwm >> 8;
	fr.data[2] = msg->left_pwm & 0x00FF;
	fr.data[3] = msg->right_pwm >> 8;
	fr.data[4] = msg->right_pwm & 0x00FF;

	can_pub.publish(fr);


}

void CanInterfaceNode::STMSetPose(const robot_interface::Point::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 6;
	fr.id = STM_ID;
	fr.data[0] = SET_POS;
	fr.data[1] = msg->pos_x >> 8;
	fr.data[2] = msg->pos_x & 0x00FF;
	fr.data[3] = msg->pos_y >> 8;
	fr.data[4] = msg->pos_y & 0x00FF;
	fr.data[5] = msg->angle >> 8;
	fr.data[6] = msg->angle & 0x00FF;

	//publish

}

void CanInterfaceNode::STMSetParam(const robot_interface::STMParam::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 6;
	fr.id = STM_ID;
	fr.data[0] = SET_PARAM;
	fr.data[1] = msg->max_linear_speed >> 8;
	fr.data[2] = msg->max_linear_speed & 0x00FF;
	fr.data[3] = msg->max_angular_speed >> 8;
	fr.data[4] = msg->max_angular_speed & 0x00FF;
	fr.data[5] = msg->max_acc >> 8;
	fr.data[6] = msg->max_acc & 0x00FF;

	can_pub.publish(fr);


}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "can_node");

	ros::NodeHandle nmh;

	CanInterfaceNode node (&nmh);

	ros::spin();
}
