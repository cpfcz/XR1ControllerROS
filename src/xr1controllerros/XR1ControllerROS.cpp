#include "XR1ControllerROS.h"



#define PI 3.141592654

XR1ControllerROS::XR1ControllerROS()
{

	XR1_ptr = new XR1ControllerPM();

	simulationTime = 0;

	//----------------------Pubs------------------------------------------------------
	SimulationStartPublisher                =  nh.advertise<std_msgs::Bool>("/startSimulation", 1);

	SimulationStopPublisher                  =  nh.advertise<std_msgs::Bool>("/stopSimulation", 1);

	LeftHandPositionPublisher    			= nh.advertise<xr1controllerros::HandMsgs>("/LeftHand/TargetPosition", 10);

	RightHandPositionPublisher   			= nh.advertise<xr1controllerros::HandMsgs>("/RightHand/TargetPosition", 10);

	MainBodyModeChangePublisher             = nh.advertise<xr1controllerros::ChainModeChange>("/XR1/MainBodyChainModeChange", 10);

	LeftArmModeChangePublisher              = nh.advertise<xr1controllerros::ChainModeChange>("/XR1/LeftArmChainModeChange", 10);

	RightArmModeChangePublisher             = nh.advertise<xr1controllerros::ChainModeChange>("/XR1/RightArmChainModeChange", 10);


	LeftHandModeChangePublisher              = nh.advertise<xr1controllerros::ChainModeChange>("/XR1/LeftHandChainModeChange", 10);

	RightHandModeChangePublisher             = nh.advertise<xr1controllerros::ChainModeChange>("/XR1/RightHandChainModeChange", 10);


	LeftArmEEFPositionPublisher                = nh.advertise<geometry_msgs::Twist>("/LeftArm/IK_msg", 10);
	RightArmEEFPositionPublisher               = nh.advertise<geometry_msgs::Twist>("/RightArm/IK_msg", 10);

	TwistPublisher                           = nh.advertise<geometry_msgs::Twist>("/XR1/Base/cmd", 10);

	LeftArmPositionPublisher                  = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/TargetPosition", 1);

	RightArmPositionPublisher                 = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/TargetPosition", 1);

	MainBodyPositionPublisher                 = nh.advertise<xr1controllerros::BodyMsgs>("/MainBody/TargetPosition", 1);

	LeftArmVelocityPublisher                  = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/TargetVelocity", 1);
	RightArmVelocityPublisher                 = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/TargetVelocity", 1);

	LeftArmCurrentPublisher                    = nh.advertise<xr1controllerros::ArmMsgs>("/LeftArm/TargetCurrent", 1);
	RightArmCurrentPublisher                  = nh.advertise<xr1controllerros::ArmMsgs>("/RightArm/TargetCurrent", 1);

	PlatformPublisher                        = nh.advertise<geometry_msgs::Twist>("/XR1/PlatformTwist" , 1 );

	LeftHandMotionPlanningMethodPublisher 	= nh.advertise<std_msgs::Int32>("/LeftArm/MotionPlanningType", 1 );

	RightHandMotionPlanningMethodPublisher 	= nh.advertise<std_msgs::Int32>("/RightArm/MotionPlanningType", 1 );

	LeftHandMotionPlanningTriggerPublisher 	= nh.advertise<std_msgs::Bool>("/LeftArm/TriggerMotionPlanning", 1 );

	RightHandMotionPlanningTriggerPublisher = nh.advertise<std_msgs::Bool>("/RightArm/TriggerMotionPlanning", 1 );

	JointAttributePublisher 				= nh.advertise<xr1controllerros::JointAttributeMsgs>("XR1/JointAttribute", 1);

	// -------------------- Subs--------------------------------------------------
	SyncModePublisher                        = nh.advertise<std_msgs::Bool>("enableSyncMode" , 1);
	TriggerPublisher                         = nh.advertise<std_msgs::Bool>("triggerNextStep" , 1);

	MainBodyPositionSubscriber               = nh.subscribe("/MainBody/Position" , 100 , &XR1ControllerROS::subscribeMainBodyPosition, this);

	LeftArmPositionSubscriber                = nh.subscribe("/LeftArm/Position" ,  100 , &XR1ControllerROS::subscribeLeftArmPosition, this);
	LeftArmVelocitySubscriber                = nh.subscribe("/LeftArm/Velocity" ,  100 , &XR1ControllerROS::subscribeLeftArmVelocity, this);
	LeftArmCurrentSubscriber                 = nh.subscribe("/LeftArm/Current" ,   100 , &XR1ControllerROS::subscribeLeftArmCurrent, this);

	RightArmPositionSubscriber               = nh.subscribe("/RightArm/Position" , 100 , &XR1ControllerROS::subscribeRightArmPosition, this);
	RightArmVelocitySubscriber               = nh.subscribe("/RightArm/Velocity" , 100 , &XR1ControllerROS::subscribeRightArmVelocity, this);
	RightArmCurrentSubscriber                = nh.subscribe("/RightArm/Current" ,  100 , &XR1ControllerROS::subscribeRightArmCurrent, this);

	simulationTimeSubscriber                 = nh.subscribe("/simulationTime" , 1      , &XR1ControllerROS::simulationTimeCallback , this);

	BaseRotSubscriber                        = nh.subscribe("/XR1/Base/Rot" , 2        , &XR1ControllerROS::tiltCompensate , this);

	ControlModes[XR1::LeftArm] = 			XR1::PositionMode;
	ControlModes[XR1::RightArm] = 			XR1::PositionMode;

	ControlModes[XR1::LeftHand] = 			XR1::PositionMode;
	ControlModes[XR1::RightHand] = 			XR1::PositionMode;

	ControlModes[XR1::MainBody] = 			XR1::PositionMode;

}

XR1ControllerROS::~XR1ControllerROS()
{
	// unregister all publishers here
	SimulationStartPublisher               .shutdown();
	SimulationPausePublisher               .shutdown();
	SimulationStopPublisher                .shutdown();
	LeftHandPositionPublisher  			   .shutdown();
	RightHandPositionPublisher 			   .shutdown();
	MainBodyModeChangePublisher            .shutdown();
	LeftArmModeChangePublisher             .shutdown();
	RightArmModeChangePublisher            .shutdown();
	IKTargetPositionPublisher              .shutdown();
	TwistPublisher                         .shutdown();
	LeftArmPositionPublisher               .shutdown();
	RightArmPositionPublisher              .shutdown();
	MainBodyPositionPublisher              .shutdown();
	LeftArmVelocityPublisher               .shutdown();
	RightArmVelocityPublisher              .shutdown();
	LeftArmCurrentPublisher                .shutdown();
	RightArmCurrentPublisher               .shutdown();
	SyncModePublisher                      .shutdown();
	TriggerPublisher                       .shutdown();
	LeftHandMotionPlanningMethodPublisher  .shutdown();
	RightHandMotionPlanningMethodPublisher .shutdown();
	LeftHandMotionPlanningTriggerPublisher .shutdown();
	RightHandMotionPlanningTriggerPublisher.shutdown();
	LeftArmEEFPositionPublisher			   .shutdown();
	RightArmEEFPositionPublisher		   .shutdown();
}


void XR1ControllerROS::launchAllMotors() {

	ControlModes[XR1::LeftArm] = XR1::PositionMode;
	ControlModes[XR1::RightArm] = XR1::PositionMode;

	ControlModes[XR1::LeftHand] = XR1::PositionMode;
	ControlModes[XR1::RightHand] = XR1::PositionMode;

	ControlModes[XR1::MainBody] = XR1::PositionMode;

	std_msgs::Bool data;

	data.data = true;

	SyncModePublisher.publish(data);

	SimulationStartPublisher.publish(data);

	simulationTime = 0;

	previous_simulationTime = 0;

	// triggerNextStep();

}

void XR1ControllerROS::startSimulation() {
	launchAllMotors();
}

void XR1ControllerROS::stopAllMotors() {
	std_msgs::Bool data;

	data.data = true;

	SimulationStopPublisher.publish(data);

	onZero();
}

void XR1ControllerROS::stopSimulation() {
	stopAllMotors() ;
}


void XR1ControllerROS::onZero() {
	XR1_ptr->Zero();
}


double XR1ControllerROS::getSimulationTime() {
	return simulationTime;
}

void XR1ControllerROS::setControlMode(u_int8_t control_group , u_int8_t option) {

	xr1controllerros::ChainModeChange mode;

	mode.ChainID = control_group;
	mode.Mode = option;

	switch (control_group) {
	case XR1::MainBody:
		MainBodyModeChangePublisher.publish(mode);
		break;

	case XR1::LeftArm:
		LeftArmModeChangePublisher.publish(mode);
		break;

	case XR1::RightArm:
		RightArmModeChangePublisher.publish(mode);
		break;

	case XR1::LeftHand:
		LeftHandModeChangePublisher.publish(mode);
		break;

	case XR1::RightHand:
		RightHandModeChangePublisher.publish(mode);
		break;

	default:
		break;
	}

	// mode.Mode = option;

	ControlModes[control_group] = option;

	// if (control_group == XR1::Actuator_Total) {
	// 	ControlModes[XR1::LeftArm] = option;
	// 	ControlModes[XR1::RightArm] = option;
	// }

	// ModeChangePublisher.publish(mode);
	// onMousePublish(ui_.publish_click_location_check_box->isChecked())
}


void XR1ControllerROS::setLeftArmPosition(const Vector3d& Linear , const Vector3d& Angular) {
	LeftArmEEFPositionPublisher.publish(ConvertIkMsgs(Linear , Angular)) ;
}

void XR1ControllerROS::setRightArmPosition(const Vector3d& Linear , const Vector3d& Angular) {
	RightArmEEFPositionPublisher.publish(ConvertIkMsgs(Linear , Angular)) ;
}

void XR1ControllerROS::setLeftArmPosition(const VectorXd& twist) {
	LeftArmEEFPositionPublisher.publish(ConvertIkMsgs(twist)) ;
}

void XR1ControllerROS::setRightArmPosition(const VectorXd& twist) {
	RightArmEEFPositionPublisher.publish(ConvertIkMsgs(twist)) ;
}


geometry_msgs::Twist XR1ControllerROS::ConvertIkMsgs(const VectorXd& twist) {

	geometry_msgs::Twist msg;

	msg.linear.x = twist(0);

	msg.linear.y = twist(1);

	msg.linear.z = twist(2);

	msg.angular.x = twist(3);

	msg.angular.y = twist(4);

	msg.angular.z = twist(5);

	return msg;
}


geometry_msgs::Twist XR1ControllerROS::ConvertIkMsgs(const Vector3d& Linear , const Vector3d& Angular) {

	geometry_msgs::Twist msg;

	msg.linear.x = Linear(0);

	msg.linear.y = Linear(1);

	msg.linear.z = Linear(2);

	msg.angular.x = Angular(0);

	msg.angular.y = Angular(1);

	msg.angular.z = Angular(2);

	return msg;
}

geometry_msgs::Twist XR1ControllerROS::ConvertIkMsgs(std::vector<double> Linear , std::vector<double> Angular) {

	geometry_msgs::Twist msg;

	msg.linear.x  = Linear[0];

	msg.linear.y  = Linear[1];

	msg.linear.z  = Linear[2] ;

	msg.angular.x = Angular[0];

	msg.angular.y = Angular[1];

	msg.angular.z = Angular[2];

	return msg;
}

xr1controllerros::HandMsgs XR1ControllerROS::ConvertHandMsgs(Eigen::VectorXd HandPosition) {

	xr1controllerros::HandMsgs msg;
	msg.Thumb = HandPosition(0);
	msg.Index = HandPosition(1);
	msg.Middle = HandPosition(2);
	msg.Ring = HandPosition(3);
	msg.Pinky = HandPosition(4);

	return msg;
}

xr1controllerros::HandMsgs XR1ControllerROS::ConvertHandMsgs(std::vector<double> HandPosition) {
	xr1controllerros::HandMsgs msg;
	msg.Thumb = HandPosition[0];
	msg.Index = HandPosition[1];
	msg.Middle = HandPosition[2];
	msg.Ring = HandPosition[3];
	msg.Pinky = HandPosition[4];
	return msg;
}


void XR1ControllerROS::setOmniVelocity(Eigen::Vector3d Linear , Eigen::Vector3d Angular) {
	// Send out a Twist msg
	geometry_msgs::Twist msg;
	msg.linear.x = Linear(0);
	msg.linear.y = Linear(1);
	msg.angular.z = Angular(2);
	TwistPublisher.publish(msg);

}

void XR1ControllerROS::setLeftArmVelocity(const Vector3d& Linear , const Vector3d& Angular) {

	XR1_ptr->setLeftArmVelocity(Linear , Angular);

}

void XR1ControllerROS::setLeftArmIncrement(const Vector3d &Linear, const Vector3d &Angular) {

	XR1_ptr->setLeftArmIncrement(Linear , Angular);

}

void XR1ControllerROS::setRightArmIncrement(const Vector3d &Linear, const Vector3d &Angular) {
	XR1_ptr->setRightArmIncrement(Linear , Angular);
}

void XR1ControllerROS::setRightArmVelocity(const Vector3d& Linear , const Vector3d& Angular) {

	XR1_ptr->setRightArmVelocity(Linear , Angular);
}

void XR1ControllerROS::setLeftArmCurrent(const Vector3d& Force , const Vector3d& Torque) {

	XR1_ptr->setLeftArmCurrent(Force , Torque);

}

void XR1ControllerROS::setRightArmCurrent(const Vector3d& Force , const Vector3d& Torque) {

	XR1_ptr->setRightArmCurrent(Force , Torque);

}


void XR1ControllerROS::setLeftArmVelocity(const VectorXd& twist) {
	XR1_ptr->setLeftArmVelocity(twist);
}

void XR1ControllerROS::setRightArmVelocity(const VectorXd& twist) {
	XR1_ptr->setRightArmVelocity(twist);
}
void XR1ControllerROS::setLeftArmIncrement(const VectorXd& twist) {
	XR1_ptr->setLeftArmIncrement(twist);
}

void XR1ControllerROS::setRightArmIncrement(const VectorXd& twist) {
	XR1_ptr->setRightArmIncrement(twist);
}

void XR1ControllerROS::setLeftArmCurrent(const VectorXd& twist) {
	XR1_ptr->setLeftArmCurrent(twist);
}

void XR1ControllerROS::setRightArmCurrent(const VectorXd& twist) {
	XR1_ptr->setRightArmCurrent(twist);
}


void XR1ControllerROS::setJointPosition(u_int8_t control_group , VectorXd JA) {

	XR1_ptr->setJointPosition(control_group , JA);

}

void XR1ControllerROS::setJointVelocity(u_int8_t control_group , VectorXd JV) {

	XR1_ptr->setJointVelocity( control_group,  JV);

}

void XR1ControllerROS::setJointCurrent(u_int8_t control_group , VectorXd JC) {

	XR1_ptr->setJointCurrent( control_group,  JC);
}

void XR1ControllerROS::updatingCallback(VectorXd JointValue, u_int8_t control_group , u_int8_t values_type) {
	XR1_ptr->updatingCallback(JointValue, control_group , values_type);
}

MatrixXd XR1ControllerROS::getJacobian(u_int8_t joint_idx) {
	return XR1_ptr->getJacobian(joint_idx);
}

std::vector<MatrixXd> XR1ControllerROS::getJacobian(std::vector<u_int8_t> joint_idx_list) {
	return XR1_ptr->getJacobian(joint_idx_list);
}


xr1controllerros::ArmMsgs XR1ControllerROS::ConvertArmMsgs(std::vector<double> input) {

	xr1controllerros::ArmMsgs msg;

	msg.Shoulder_X = input[0];
	msg.Shoulder_Y = input[1];
	msg.Elbow_Z = input[2];
	msg.Elbow_X = input[3];
	msg.Wrist_Z = input[4];
	msg.Wrist_Y = input[5];
	msg.Wrist_X = input[6];

	return msg;
}

xr1controllerros::ArmMsgs XR1ControllerROS::ConvertArmMsgs(Eigen::VectorXd input) {
	xr1controllerros::ArmMsgs msg;

	msg.Shoulder_X = input(0);
	msg.Shoulder_Y = input(1);
	msg.Elbow_Z = input(2);
	msg.Elbow_X = input(3);
	msg.Wrist_Z = input(4);
	msg.Wrist_Y = input(5);
	msg.Wrist_X = input(6);

	return msg;
}

xr1controllerros::BodyMsgs XR1ControllerROS::ConvertBodyMsgs(std::vector<double> input) {

	xr1controllerros::BodyMsgs msg;

	msg.Knee   = input[0];
	msg.Back_Z = input[1];
	msg.Back_X = input[2];
	msg.Back_Y = input[3];
	msg.Neck_Z = input[4];
	msg.Neck_X = input[5];
	msg.Head = input[6];

	return msg;
}

xr1controllerros::BodyMsgs XR1ControllerROS::ConvertBodyMsgs(Eigen::VectorXd input) {

	xr1controllerros::BodyMsgs msg;

	msg.Knee   = input(0);
	msg.Back_Z = input(1);
	msg.Back_X = input(2);
	msg.Back_Y = input(3);
	msg.Neck_Z = input(4);
	msg.Neck_X = input(5);
	msg.Head = input(6);

	return msg;
}

Eigen::VectorXd XR1ControllerROS::BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs& msg) {

	Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

	res << msg.Knee  ,
	    msg.Back_Z,
	    msg.Back_X,
	    msg.Back_Y,
	    msg.Neck_Z,
	    msg.Neck_X,
	    msg.Head;

	return res;
}

Eigen::VectorXd XR1ControllerROS::ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs& msg) {

	Eigen::VectorXd res = Eigen::VectorXd::Zero(7);

	res << msg.Shoulder_X ,
	    msg.Shoulder_Y,
	    msg.Elbow_Z ,
	    msg.Elbow_X ,
	    msg.Wrist_Z ,
	    msg.Wrist_Y ,
	    msg.Wrist_X ;

	return res;
}

Eigen::VectorXd XR1ControllerROS::HandsMsgs2VectorXd(const xr1controllerros::HandMsgs& msg) {

	Eigen::VectorXd res = Eigen::VectorXd::Zero(5);

	res << msg.Thumb  ,
	    msg.Index,
	    msg.Middle,
	    msg.Ring,
	    msg.Pinky;


	return res;
}


void XR1ControllerROS::subscribeLeftArmVelocity(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::LeftArm , XR1::ActualVelocity);
}

void XR1ControllerROS::subscribeLeftArmCurrent(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::LeftArm , XR1::ActualCurrent);
}

void XR1ControllerROS::subscribeRightArmVelocity(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::RightArm , XR1::ActualVelocity);
}

void XR1ControllerROS::subscribeRightArmCurrent(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::RightArm , XR1::ActualCurrent);
}

void XR1ControllerROS::subscribeLeftArmPosition(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::LeftArm , XR1::ActualPosition);
}

void XR1ControllerROS::subscribeRightArmPosition(const xr1controllerros::ArmMsgs& msg) {
	XR1_ptr->updatingCallback(ArmMsgs2VectorXd(msg) , XR1::RightArm , XR1::ActualPosition);
}

void XR1ControllerROS::subscribeMainBodyPosition(const xr1controllerros::BodyMsgs& msg) {
	XR1_ptr->updatingCallback(BodyMsgs2VectorXd(msg) , XR1::LeftArm , XR1::ActualPosition);
}

void XR1ControllerROS::subscribeLeftHandPosition(const xr1controllerros::HandMsgs& msg) {
	XR1_ptr->updatingCallback(HandsMsgs2VectorXd(msg) , XR1::LeftHand , XR1::ActualPosition);
}

void XR1ControllerROS::subscribeRightHandPosition(const xr1controllerros::HandMsgs& msg) {
	XR1_ptr->updatingCallback(HandsMsgs2VectorXd(msg) , XR1::RightHand , XR1::ActualPosition);
}

void XR1ControllerROS::tiltCompensate(const geometry_msgs::Quaternion& msg) {

	Eigen::Quaterniond q(msg.w, msg.x , msg.y , msg.z);

	XR1_ptr->TiltCompensation(q);
}

void XR1ControllerROS::triggerNextStep() {
	std_msgs::Bool msg;

	msg.data = true;

	TriggerPublisher.publish(msg);
}

void XR1ControllerROS::simulationTimeCallback(const std_msgs::Float32& msg) {
	simulationTime = msg.data;
}


void XR1ControllerROS::setPathPlanningMethod(u_int8_t control_group , u_int8_t method) {

}


void XR1ControllerROS::setJointAttribute(u_int8_t joint_idx , u_int8_t attribute_idx, double value) {
	xr1controllerros::JointAttributeMsgs msg;
	msg.JointID = joint_idx;
	msg.AttributeID = attribute_idx;
	msg.Value = value;

	JointAttributePublisher.publish(msg);
}



void XR1ControllerROS::setLeftArmDynamicSwitch(bool option) {
	XR1_ptr->setLeftArmDynamicSwitch(option);
}
void XR1ControllerROS::setRightArmDynamicSwitch(bool option) {
	XR1_ptr->setRightArmDynamicSwitch(option);
}




VectorXd XR1ControllerROS::getJointPositions(u_int8_t control_group) {
	return XR1_ptr->getJointPositions(control_group);
}



std::vector<double> XR1ControllerROS::getJointPositionsStd(u_int8_t control_group) {
	return XR1_ptr->getJointPositionsStd(control_group);
}



VectorXd XR1ControllerROS::getJointVelocities(u_int8_t control_group) {

	return XR1_ptr->getJointVelocities(control_group);
}


std::vector<double> XR1ControllerROS::getJointVelocitiesStd(u_int8_t control_group) {
	return XR1_ptr->getJointVelocitiesStd(control_group);
}



VectorXd XR1ControllerROS::getJointCurrents(u_int8_t control_group) {
	return XR1_ptr->getJointCurrents(control_group);
}



std::vector<double> XR1ControllerROS::getJointCurrentsStd(u_int8_t control_group) {
	return XR1_ptr->getJointCurrentsStd(control_group);
}


VectorXd XR1ControllerROS::getLeftArmForce() {
	return XR1_ptr->getLeftArmForce();
}

VectorXd XR1ControllerROS::getLeftArmVelocity() {
	return XR1_ptr->getLeftArmVelocity();
}

VectorXd XR1ControllerROS::getLeftArmPosition() {
	return XR1_ptr->getLeftArmPosition();
}

MatrixXd XR1ControllerROS::getLeftArmPositionMatrix() {
	return XR1_ptr->getLeftArmPositionMatrix();
}

VectorXd XR1ControllerROS::getRightArmForce() {
	return XR1_ptr->getRightArmForce();
}

VectorXd XR1ControllerROS::getRightArmVelocity() {
	return XR1_ptr->getRightArmVelocity();
}

VectorXd XR1ControllerROS::getRightArmPosition() {
	return XR1_ptr->getRightArmPosition();
}

MatrixXd XR1ControllerROS::getRightArmPositionMatrix() {
	return XR1_ptr->getRightArmPositionMatrix();
}

void XR1ControllerROS::stepFinishedCallback() {


	switch (ControlModes[XR1::LeftArm]) {
	case XR1::PositionMode:
		LeftArmPositionPublisher.publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::LeftArm)));
		break;
	case XR1::VelocityMode:

		LeftArmVelocityPublisher.publish(ConvertArmMsgs(XR1_ptr->getTargetVelocity(XR1::LeftArm)));
		break;
	case XR1::ForceMode:
		LeftArmCurrentPublisher.publish(ConvertArmMsgs(XR1_ptr->getTargetCurrent(XR1::LeftArm)));
		break;
	default:
		break;
	}

	switch (ControlModes[XR1::RightArm]) {
	case XR1::PositionMode:
		RightArmPositionPublisher.publish(ConvertArmMsgs(XR1_ptr->getTargetPosition(XR1::RightArm)));
		break;
	case XR1::VelocityMode:

		RightArmVelocityPublisher.publish(ConvertArmMsgs(XR1_ptr->getTargetVelocity(XR1::RightArm)));
		break;
	case XR1::ForceMode:
		RightArmCurrentPublisher.publish(ConvertArmMsgs(XR1_ptr->getTargetCurrent(XR1::RightArm)));
		break;
	default:
		break;
	}


	switch (ControlModes[XR1::LeftHand]) {
	case XR1::PositionMode:
		LeftHandPositionPublisher.publish(ConvertHandMsgs(XR1_ptr->getTargetPosition(XR1::LeftHand)));
		break;
	case XR1::VelocityMode:
		// LeftHandVelocityPublisher.publish(ConvertHandMsgs(XR1_ptr->getTargetVelocity(XR1::LeftHand)));
		break;
	case XR1::ForceMode:
		LeftHandCurrentPublisher.publish(ConvertHandMsgs(XR1_ptr->getTargetCurrent(XR1::LeftHand)));
		break;
	default:
		break;
	}


	switch (ControlModes[XR1::RightHand]) {
	case XR1::PositionMode:
		RightHandPositionPublisher.publish(ConvertHandMsgs(XR1_ptr->getTargetPosition(XR1::RightHand)));
		break;
	case XR1::VelocityMode:
		// RightHandVelocityPublisher.publish(ConvertHandMsgs(XR1_ptr->getTargetVelocity(XR1::RightHand)));
		break;
	case XR1::ForceMode:
		RightHandCurrentPublisher.publish(ConvertHandMsgs(XR1_ptr->getTargetCurrent(XR1::RightHand)));
		break;
	default:
		break;
	}

	MainBodyPositionPublisher.publish(ConvertBodyMsgs(XR1_ptr->getTargetPosition(XR1::MainBody)));


	triggerNextStep();
}
