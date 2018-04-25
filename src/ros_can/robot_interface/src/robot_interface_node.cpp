#include <robot_interface/robot_interface_node.h>
#include <iterator>


CanInterfaceNode::CanInterfaceNode(ros::NodeHandle *n){
	this->nh = *n;

	this->can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 1000);

	this->STM_coder_pub = nh.advertise<can_msgs::WheelsDistance>("/STM/GetCoder", 1);
	this->STM_pos_pub = nh.advertise<can_msgs::Point>("/STM/Position", 1);
	this->STM_pwm_pub = nh.advertise<can_msgs::PWMs>("/STM/GetPWM", 1);
	this->STM_speed_pub = nh.advertise<can_msgs::CurrSpeed>("/STM/GetSpeed", 1);
	this->STM_speed_pub = nh.advertise<can_msgs::CurrSpeed>("/STM/GetSpeed", 1);
	this->ALL_finish_pub = nh.advertise<can_msgs::Finish>("/ALL/Finish", 1);

	this->robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_watcher", 1, &CanInterfaceNode::updateRobotStatus, this);
	this->can_sub = nh.subscribe("received_messages", 1, &CanInterfaceNode::canMsgProcess, this);

	this->STMSetMode_sub = nh.subscribe("/STM/SetMode",1, &CanInterfaceNode::STMSetMode, this);
	this->STMSpeed_sub = nh.subscribe("/STM/Speed",1, &CanInterfaceNode::STMSpeed, this);
	this->STMAsserManagement_sub = nh.subscribe("/STM/AsserManagement",1, &CanInterfaceNode::STMAsserManagement, this);
	this->STMGoToAngle_sub = nh.subscribe("/STM/GoToAngle",1, &CanInterfaceNode::STMGoToAngle, this);
	this->STMGoTo_sub = nh.subscribe("/STM/GoTo",1, &CanInterfaceNode::STMGoTo, this);
	this->STMRotation_sub = nh.subscribe("/STM/Rotation",1, &CanInterfaceNode::STMRotation, this);
	this->STMRotationNoModulo_sub = nh.subscribe("/STM/RotationNoModulo",1, &CanInterfaceNode::STMRotationNoModulo, this);
	this->STMLeftPID_sub = nh.subscribe("/STM/LeftPID",1, &CanInterfaceNode::STMLeftPID, this);
	this->STMRightPID_sub = nh.subscribe("/STM/RightPID",1, &CanInterfaceNode::STMRightPID, this);
	this->STMAllPID_sub = nh.subscribe("/STM/AllPID",1, &CanInterfaceNode::STMAllPID, this);
	this->STMPWM_sub = nh.subscribe("/STM/PWM",1, &CanInterfaceNode::STMPWM, this);
	this->STMSetPose_sub = nh.subscribe("/STM/SetPose",1, &CanInterfaceNode::STMSetPose, this);
	this->STMSetParam_sub = nh.subscribe("/STM/SetParam",1, &CanInterfaceNode::STMSetParam, this);

	service_ready("ros_can", "interface", 1 );

}

CanInterfaceNode::~CanInterfaceNode(){

}

void CanInterfaceNode::updateRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
	this->robot_watcher = msg->robot_watcher;
	// ROS_INFO("callback robot_watcher: %d", this->robot_watcher);
}

void CanInterfaceNode::canMsgProcess(const can_msgs::Frame::ConstPtr& msg){

	switch (msg->data[0]) {
		case WHOAMI:{

			uint boardID = msg->id;
			uint8_t status = msg->data[1];
			break;
			}
		case GET_CODER:{
			can_msgs::WheelsDistance msg_out;
			msg_out.right_wheel_dist = msg->data[2] | msg->data[1] << 8; //mm
			msg_out.left_wheel_dist = msg->data[4] | msg->data[3] << 8; //mm

			this->STM_coder_pub.publish(msg_out);
			break;
		}
		case CURRENT_POS:{
		can_msgs::Point msg_out;

			msg_out.pos_x = msg->data[2] | msg->data[1] << 8; //mm
			msg_out.pos_y = msg->data[4] | msg->data[3] << 8; //mm
			msg_out.angle = msg->data[6] | msg->data[5] << 8; //mRad

			this->STM_pos_pub.publish(msg_out);
			break;
		}
		case CURRENT_PWM:{
			can_msgs::PWMs msg_out;

			msg_out.left_pwm = msg->data[2] | msg->data[1] << 8;
			msg_out.right_pwm = msg->data[4] | msg->data[3] << 8;

			this->STM_pwm_pub.publish(msg_out);
			break;
		}
		case CURRENT_SPD:{
			can_msgs::CurrSpeed msg_out;

			msg_out.linear_speed = msg->data[2] | msg->data[1] << 8; //mm/s
			msg_out.left_speed = msg->data[4] | msg->data[3] << 8; //mm/s
			msg_out.right_speed = msg->data[6] | msg->data[5] << 8; //mm/s

			this->STM_speed_pub.publish(msg_out);
			break;
		}
			case ORDER_COMPLETED:{
				can_msgs::Finish msg_out;

				msg_out.val = msg->data[1];

				this->ALL_finish_pub.publish(msg_out);
				break;
		}
	}

}


void CanInterfaceNode::STMSetMode(const can_msgs::Status::ConstPtr& msg){
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

void CanInterfaceNode::STMSpeed(const can_msgs::Speed::ConstPtr& msg){

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

void CanInterfaceNode::STMAsserManagement(const can_msgs::Status::ConstPtr& msg){
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

void CanInterfaceNode::STMGoToAngle(const can_msgs::Point::ConstPtr& msg){
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

void CanInterfaceNode::STMGoTo(const can_msgs::Point::ConstPtr& msg){
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

void CanInterfaceNode::STMRotation(const can_msgs::Point::ConstPtr& msg){
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

void CanInterfaceNode::STMRotationNoModulo(const can_msgs::Point::ConstPtr& msg){
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

void CanInterfaceNode::STMLeftPID(const can_msgs::PID::ConstPtr& msg){
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

void CanInterfaceNode::STMRightPID(const can_msgs::PID::ConstPtr& msg){
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

void CanInterfaceNode::STMAllPID(const can_msgs::PID::ConstPtr& msg){
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

void CanInterfaceNode::STMPWM(const can_msgs::PWMs::ConstPtr& msg){
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

void CanInterfaceNode::STMSetPose(const can_msgs::Point::ConstPtr& msg){
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

void CanInterfaceNode::STMSetParam(const can_msgs::STMParam::ConstPtr& msg){
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
