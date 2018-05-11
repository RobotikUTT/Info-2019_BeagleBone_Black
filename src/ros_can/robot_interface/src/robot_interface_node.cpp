#include <robot_interface/robot_interface_node.h>
#include <iterator>


CanInterfaceNode::CanInterfaceNode(ros::NodeHandle *n){
	this->nh = *n;

	this->can_pub 										= nh.advertise<can_msgs::Frame>("sent_messages", 									1000);

	this->STM_coder_pub 							= nh.advertise<can_msgs::WheelsDistance>("/STM/GetCoder", 				10);
	this->STM_pos_pub 								= nh.advertise<can_msgs::Point>("/STM/Position", 									10);
	this->STM_pwm_pub 								= nh.advertise<can_msgs::PWMs>("/STM/GetPWM", 										10);
	this->STM_speed_pub 							= nh.advertise<can_msgs::CurrSpeed>("/STM/GetSpeed", 							10);
	this->ALL_finish_pub	 						= nh.advertise<can_msgs::Finish>("/ALL/Finish", 									10);
	this->ARDUINO_sonar_distance_pub 	= nh.advertise<can_msgs::SonarDistance>("/ARDUINO/SonarDistance",	10);
	this->STM_robot_blocked_pub 			= nh.advertise<can_msgs::RobotBlocked>("/STM/RobotBlocked", 			10);
	//this->LIDAR_object_on_map_pub = nh.advertise<can_msgs::ObjectOnMap>("LIDAR/ObjectOnMap",10);

	this->robot_watcher_sub 					= nh.subscribe("/ai/robot_watcher/robot_watcher", 10, &CanInterfaceNode::updateRobotStatus, this);
	this->can_sub 										= nh.subscribe("received_messages", 							100, &CanInterfaceNode::canMsgProcess, 		this);

	this->ALL_Ping_sub 								= nh.subscribe("/ALL/Ping",												10, &CanInterfaceNode::ALLPing, 						this);
	this->STM_SetMode_sub 						= nh.subscribe("/STM/SetMode",										10, &CanInterfaceNode::STMSetMode, 					this);
	this->STM_Speed_sub 							= nh.subscribe("/STM/Speed",											10, &CanInterfaceNode::STMSpeed, 						this);
	this->STM_AsserManagement_sub 		= nh.subscribe("/STM/AsserManagement",						10, &CanInterfaceNode::STMAsserManagement, 	this);
	this->STM_GoToAngle_sub 					= nh.subscribe("/STM/GoToAngle",									10, &CanInterfaceNode::STMGoToAngle, 				this);
	this->STM_GoTo_sub 								= nh.subscribe("/STM/GoTo",												10, &CanInterfaceNode::STMGoTo, 						this);
	this->STM_Rotation_sub 						= nh.subscribe("/STM/Rotation",										10, &CanInterfaceNode::STMRotation, 				this);
	this->STM_RotationNoModulo_sub 		= nh.subscribe("/STM/RotationNoModulo",						10, &CanInterfaceNode::STMRotationNoModulo, this);
	this->STM_LeftPID_sub 						= nh.subscribe("/STM/LeftPID",										10, &CanInterfaceNode::STMLeftPID, 					this);
	this->STM_RightPID_sub 						= nh.subscribe("/STM/RightPID",										10, &CanInterfaceNode::STMRightPID, 				this);
	this->STM_AllPID_sub 							= nh.subscribe("/STM/AllPID",											10, &CanInterfaceNode::STMAllPID, 					this);
	this->STM_PWM_sub									= nh.subscribe("/STM/PWM",												10, &CanInterfaceNode::STMPWM, 							this);
	this->STM_SetPose_sub 						= nh.subscribe("/STM/SetPose",										10, &CanInterfaceNode::STMSetPose, 					this);
	this->STM_SetParam_sub 						= nh.subscribe("/STM/SetParam",										10, &CanInterfaceNode::STMSetParam, 				this);
	this->ARDUINO_ActionPliers_sub 		= nh.subscribe("/ARDUINO/ActionPliers",						10, &CanInterfaceNode::ARDUINOActionPliers, this);
	this->ARDUINO_MovePliers_sub 			= nh.subscribe("/ARDUINO/MovePliers",							10, &CanInterfaceNode::ARDUINOMovePliers, this);
	this->ARDUINO_ThrowBalls_sub 			= nh.subscribe("/ARDUINO/ThrowBalls",							10, &CanInterfaceNode::ARDUINOThrowBalls, 	this);
	this->PANEL_point_sub 						= nh.subscribe("/PANEL/AddPoint",									10, &CanInterfaceNode::PANELAddPoint, 			this);

	service_ready("ros_can", "interface", 1 );
}

CanInterfaceNode::~CanInterfaceNode(){
}

void CanInterfaceNode::updateRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
	this->robot_watcher = msg->robot_watcher;
	// ROS_INFO("callback robot_watcher: %d", this->robot_watcher);
}

void CanInterfaceNode::canMsgProcess(const can_msgs::Frame::ConstPtr& msg){
	if (msg->id == BBB_CAN_ADDR){
	switch (msg->data[0]) {
		case WHOAMI:{

			uint8_t status = msg->data[2];

			std::string boardName;

			switch (msg->data[1]) {
				case STM_CAN_ADDR:{
					boardName = "STM";
					break;
				}
				case ARDUINO_CAN_ADDR:{
					boardName = "ARDUINO";
					break;
				}
				case ZIGBEE_CAN_ADDR:{
					boardName = "ZIGBEE";
					break;
				}
				case PANEL_CAN_ADDR:{
					boardName = "PANEL";
					break;
				}
			}

			service_ready("board", boardName, status );
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
		case SONAR_DISTANCE:{
			can_msgs::SonarDistance msg_out;

			msg_out.dist_front_left 	= msg->data[1];
			msg_out.dist_front_right 	= msg->data[2];
			msg_out.dist_back_left  	= msg->data[3];
			msg_out.dist_back_right 	= msg->data[4];

			this->ARDUINO_sonar_distance_pub.publish(msg_out);
			break;
		}
		case ORDER_COMPLETED:{
			can_msgs::Finish msg_out;

			msg_out.val = msg->data[1];

			this->ALL_finish_pub.publish(msg_out);
			break;
		}
		case ROBOT_BLOCKED:{
			can_msgs::RobotBlocked msg_out;

			this->STM_robot_blocked_pub.publish(msg_out);
			break;
		}
	}
	}
}

void CanInterfaceNode::ALLPing(const std_msgs::Empty::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 1;
	fr.id = ALL_CAN_ADDR;
	fr.data[0] = HANDSHAKE;

	can_pub.publish(fr);
}

void CanInterfaceNode::STMSetMode(const can_msgs::Status::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 2;
	fr.id = STM_CAN_ADDR;
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
	fr.id = STM_CAN_ADDR;
	fr.data[0] = SPD;
	fr.data[1] = msg->linear_speed >> 8;
	fr.data[2] = msg->linear_speed & 0x00FF;
	fr.data[3] = msg->angular_speed >> 8;
	fr.data[4] = msg->angular_speed & 0x00FF;
	fr.data[5] = msg->duration >> 8;
	fr.data[6] = msg->duration & 0x00FF;

	can_pub.publish(fr);
}


void CanInterfaceNode::PANELAddPoint(const std_msgs::Int8::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 2;
	fr.id = PANEL_CAN_ADDR;
	fr.data[0] = SEND_POINT;
	fr.data[1] = msg->data;

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
	fr.id = STM_CAN_ADDR;
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
	fr.id = STM_CAN_ADDR;
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
	fr.id = STM_CAN_ADDR;
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
	fr.id = STM_CAN_ADDR;
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
	fr.id = STM_CAN_ADDR;
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
	fr.id = STM_CAN_ADDR;
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
	fr.id = STM_CAN_ADDR;
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
	fr.id = STM_CAN_ADDR;
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
	fr.id = STM_CAN_ADDR;
	fr.data[0] = PWM;
	fr.data[1] = msg->left_pwm >> 8;
	fr.data[2] = msg->left_pwm & 0x00FF;
	fr.data[3] = msg->right_pwm >> 8;
	fr.data[4] = msg->right_pwm & 0x00FF;

	can_pub.publish(fr);
}

void CanInterfaceNode::STMSetPose(const can_msgs::Point::ConstPtr& msg){
	ROS_INFO_STREAM("SET POSE X: " << msg->pos_x);
	ROS_INFO_STREAM("SET POSE Y: " << msg->pos_y);
	ROS_INFO_STREAM("SET ANGLE: " << msg->angle);
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 7;
	fr.id = STM_CAN_ADDR;
	fr.data[0] = SET_POS;
	fr.data[1] = msg->pos_x >> 8;
	fr.data[2] = msg->pos_x & 0x00FF;
	fr.data[3] = msg->pos_y >> 8;
	fr.data[4] = msg->pos_y & 0x00FF;
	fr.data[5] = msg->angle >> 8;
	fr.data[6] = msg->angle & 0x00FF;

	can_pub.publish(fr);
}

void CanInterfaceNode::STMSetParam(const can_msgs::STMParam::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 7;
	fr.id = STM_CAN_ADDR;
	fr.data[0] = SET_PARAM;
	fr.data[1] = msg->max_linear_speed >> 8;
	fr.data[2] = msg->max_linear_speed & 0x00FF;
	fr.data[3] = msg->max_angular_speed >> 8;
	fr.data[4] = msg->max_angular_speed & 0x00FF;
	fr.data[5] = msg->max_acc >> 8;
	fr.data[6] = msg->max_acc & 0x00FF;

	can_pub.publish(fr);
}

void CanInterfaceNode::ARDUINOActionPliers(const can_msgs::ActionPliers::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 3;
	fr.id = ARDUINO_CAN_ADDR;
	fr.data[0] = ACTION_PLIERS;
	fr.data[1] = msg->action;
	fr.data[2] = msg->level;

	can_pub.publish(fr);
}

void CanInterfaceNode::ARDUINOMovePliers(const std_msgs::Int8::ConstPtr& msg){
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 2;
	fr.id = ARDUINO_CAN_ADDR;
	fr.data[0] = MOVE_PLIERS;
	fr.data[1] = msg->data;

	can_pub.publish(fr);
}

void CanInterfaceNode::ARDUINOThrowBalls(const can_msgs::ThrowBalls::ConstPtr& msg)
{
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface";
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;

	fr.dlc = 1;
	fr.id = ARDUINO_CAN_ADDR;
	fr.data[0] = THROW_BALLS;

	can_pub.publish(fr);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "can_node");

	ros::NodeHandle nmh;

	CanInterfaceNode node (&nmh);

	ros::spin();
}
