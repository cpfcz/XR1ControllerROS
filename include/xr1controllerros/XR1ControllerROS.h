#ifndef XR1ControllerROS_H
#define XR1ControllerROS_H

#include "ros/ros.h"

#include <geometry_msgs/Point.h>


#include "xr1controllerpm.h"
#include "xr1define.h"


// Messages for Communication
#include "xr1controllerros/ArmMsgs.h"
#include "xr1controllerros/BodyMsgs.h"
#include "xr1controllerros/HandMsgs.h"
#include "xr1controllerros/ChainModeChange.h"
#include "std_msgs/Float32.h"
#include "xr1controllerros/IK_msg.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "xr1controllerros/JointAttributeMsgs.h"
// Messages for Communication


#include "Eigen/Dense"



class XR1ControllerROS
{

public:
	XR1ControllerROS();

	~XR1ControllerROS();

	//Identical to launchAllMotors() in simulation, trigger sync mode and start the simulation
	void startSimulation();//

	//Identical to startSimulation() in simulation, trigger sync mode and start the simulation
	//Used in the XR1Controller
	void launchAllMotors();//


	//Identical to stopAllMotors() in simulation, stop the simulation
	void stopSimulation();//

	//Identical to stopSimulation() in simulation, stop the simulation
	//Used in the XR1Controller
	void stopAllMotors();//

	//Get the simulation time in double (s)
	//Used in Simulation
	double getSimulationTime();


	//--------Joint Control----------------------------------

	//Set the Joint PID values for a joint
	//Used in the XR1Controller
	//Argu: Joint ID , Attribute ID , value
	//Reutrns : void , may add error message in the fulture
	void setJointAttribute(uint8_t joint_idx , uint8_t attribute_idx , double value);


	//Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID , Angles contained in Eigen::VectorXd
	//Reutrns : void , may add error message in the fulture
	void setJointPosition(uint8_t control_group , VectorXd JA);


	//Set the Target Joint Positions for a single joint, i.e. LeftShoulderX , RightWristZ
	//Used in the XR1Controller
	//Argu: Control Group ID , Angles contained in std::vector<double>
	//Reutrns : void , may add error message in the fulture
	void setJointPosition(uint8_t joint_idx ,   double JA);


	//Set the Target Joint Velocity for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID , Angular Velocity contained in Eigen::VectorXd
	//Reutrns : void , may add error message in the fulture
	void setJointVelocity(uint8_t control_group , VectorXd JV);


	//Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
	//Used in the XR1Controller
	//Argu: Control Group ID , Angular Velocity contained in std::vector<double>
	//Reutrns : void , may add error message in the fulture
	void setJointVelocity(uint8_t joint_idx ,   double JV);


	//Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID , Target Current
	//Reutrns : void , may add error message in the fulture
	void setJointCurrent(uint8_t control_group , VectorXd JC);


	//Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
	//Used in the XR1Controller
	//Argu: Control Group ID , Angular Velocity contained in std::vector<double>
	//Reutrns : void , may add error message in the fulture
	void setJointCurrent(uint8_t joint_idx ,   double JC);


	//Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID
	//Reutrns : JointAngles contained in Eigen::VectorXd
	VectorXd getJointPositions(uint8_t control_group);


	//Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID
	//Reutrns : JointAngles contained in std::vector
	std::vector<double> getJointPositionsStd(uint8_t control_group);


	//Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID
	//Reutrns : Joint Velocities contained in Eigen::VectorXd
	VectorXd getJointVelocities(uint8_t control_group);


	//Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID
	//Reutrns : Joint Velocites contained in std::vector
	std::vector<double> getJointVelocitiesStd(uint8_t control_group);


	//Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID
	//Reutrns : Joint Currents contained in Eigen::VectorXd
	VectorXd getJointCurrents(uint8_t control_group);


	//Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID
	//Reutrns : Joint Currents contained in std::vector
	std::vector<double> getJointCurrentsStd(uint8_t control_group);


	//Set the Control Method for an entire Control Group , i.e. LeftARM , RightHand
	//Used in the XR1Controller
	//Argu: Control Group ID , Conrol Mode ID
	//Reutrns : void , may add error message in the fulture
	void setControlMode(uint8_t control_group , uint8_t option);


	// (Not Imlemented ) Set the Velocity for OmniWheels
	//Used in the XR1Controller
	//Argu: Linear Velocity contained in Eigen::Vector3d , Angular Velocity contained in Eigen::Vector3d
	//Reutrns : void , may add error message in the fulture
	void setOmniVelocity(Eigen::Vector3d Linear , Eigen::Vector3d Angular);


	// (Not Imlemented ) Set the Method For Path Planning ( Only LeftArm and RightArm)
	//
	//Argu: Control Group ID ( Only LeftArm and RightArm) ,  Method ID
	//Reutrns : void , may add error message in the fulture
	void setPathPlanningMethod(uint8_t control_group , uint8_t method);


	// Trigger the Next Simulation Step in V-REP
	//Only in simulation
	//Argu: N/A
	//Reutrns : void , may add error message in the fulture
	void triggerNextStep();


	// Send out all the target imforation to V-REP , should be called before triggerNextStep();
	//Only in simulation
	//Argu: N/A
	//Reutrns : void , may add error message in the fulture
	void stepFinishedCallback();


	//Get the last calculated Jacobian from XR1Controller
	//Used in XR1Controller
	//Argu: Joint ID (Only From LeftShoulderX to RightWristX)
	//Reutrns : A 6x7 Eigen::MatrixXd of Jacobians or A 6x7 Eigen::MatrixXd of zeros should an error occur
	MatrixXd getJacobian(uint8_t joint_idx);


	// //Get the last calculated Jacobian from XR1Controller for several joints
	// //Used in XR1Controller
	// //Argu: Vector of Joint ID (Only From LeftShoulderX to RightWristX)
	// //Reutrns : A 6x7 Eigen::MatrixXd of Jacobians or A 6x7 Eigen::MatrixXd of zeros should an error occur
	// std::vector<MatrixXd> getJacobian(std::vector<uint8_t> joint_idx_list);


	//-------------------End Effector (Wrist) Control ---------------------------

	//Move the End Effector of LeftArm for a SMALL distance
	//Used in XR1Controller
	//Argu: Linear increment contained in Eigen::Vector3d , Angular increment contained in Eigen::Vector3d
	//Reutrns : void , may add error message in the fulture
	void setLeftArmIncrement(const Vector3d& Linear , const Vector3d& Angular);
	void setLeftArmIncrement(const VectorXd& twist);

	//Move the End Effector of RightArm for a SMALL distance
	//Used in XR1Controller
	//Argu: Linear increment contained in Eigen::Vector3d , Angular increment contained in Eigen::Vector3d
	//Reutrns : void , may add error message in the fulture
	void setRightArmIncrement(const Vector3d& Linear , const Vector3d& Angular);
	void setRightArmIncrement(const VectorXd& twist);

	//Move the End Effector of LeftArm to a target position
	//Used in XR1Controller , But in simulation this moves the Target Dummy to the location
	//Argu: Position  contained in Eigen::Vector3d , Orientation (Euler Angles in the order XYZ) contained in Eigen::Vector3d
	//Reutrns : void , may add error message in the fulture
	void setLeftArmPosition(const Vector3d& Linear , const Vector3d& Angular);
	void setLeftArmPosition(const VectorXd& twist);


	//Move the End Effector of RightArm to a target position
	//Used in XR1Controller , But in simulation this moves the Target Dummy to the location
	//Argu: Position  contained in Eigen::Vector3d , Orientation (Euler Angles in the order XYZ) contained in Eigen::Vector3d
	//Reutrns : void , may add error message in the fulture
	void setRightArmPosition(const Vector3d& Linear , const Vector3d& Angular);
	void setRightArmPosition(const VectorXd& twist);

	//Set the velocity of the End Effector of LeftArm,
	//Used in XR1Controller , Needs to be refreshed every simulation step
	//Argu: Linear velocity contained in Eigen::Vector3d , Angular velocity (Euler Angles in the order XYZ) contained in Eigen::Vector3d
	//Reutrns : void , may add error message in the fulture
	void setLeftArmVelocity(const Vector3d& Linear , const Vector3d& Angular);
	void setLeftArmVelocity(const VectorXd& twist);

	//Set the velocity of the End Effector of RightArm,
	//Used in XR1Controller , Needs to be refreshed every simulation step when used
	//Argu: Linear velocity contained in Eigen::Vector3d , Angular velocity (Euler Angles in the order XYZ) contained in Eigen::Vector3d
	//Reutrns : void , may add error message in the fulture
	void setRightArmVelocity(const Vector3d& Linear , const Vector3d& Angular);
	void setRightArmVelocity(const VectorXd& twist);

	//Set the velocity of the End Effector of LeftArm , with the addition of dynamic compensation
	//Used in XR1Controller , Needs to be refreshed every simulation step when used
	//Argu: Force contained in Eigen::Vector3d , Torque contained in Eigen::Vector3d
	//Reutrns : void , may add error message in the fulture
	void setLeftArmForce(const Vector3d& Force , const Vector3d& Torque);
	void setLeftArmForce(const VectorXd& twist);

	//Set the velocity of the End Effector of RightArm , with the addition of dynamic compensation
	//Used in XR1Controller , Needs to be refreshed every simulation step when used
	//Argu: Force contained in Eigen::Vector3d , Torque contained in Eigen::Vector3d
	//Reutrns : void , may add error message in the fulture
	void setRightArmForce(const Vector3d& Force , const Vector3d& Torque);
	void setRightArmForce(const VectorXd& twist);


	//get the velocity on End Effector of the left arm
	//Used in XR1Controller , Needs to be refreshed every simulation step when used
	//Argu: N/A
	//Reutrns : VectorXd of 3 linear velocity and 3 angular velocity , may add error message in the fulture
	VectorXd getLeftArmVelocity() ;

	//get the position on End Effector of the left arm
	//Used in XR1Controller , Needs to be refreshed every simulation step when used
	//Argu: N/A
	//Reutrns : VectorXd of 3 linear position and 3 angular position , may add error message in the fulture
	VectorXd getLeftArmPosition() ;

	//get the Homogenous transformation End Effector of the left arm
	//Used in XR1Controller , Needs to be refreshed every simulation step when used
	//Argu: N/A
	//Reutrns : A homogenous transformation (4x4)
	MatrixXd getLeftArmPositionMatrix() ;


	//get the velocity on End Effector of the left arm
	//Used in XR1Controller , Needs to be refreshed every simulation step when used
	//Argu: N/A
	//Reutrns : VectorXd of 3 linear velocity and 3 angular velocity , may add error message in the fulture
	VectorXd getRightArmVelocity() ;

	//get the position on End Effector of the left arm
	//Used in XR1Controller , Needs to be refreshed every simulation step when used
	//Argu: N/A
	//Reutrns : VectorXd of 3 linear position and 3 angular position , may add error message in the fulture
	VectorXd getRightArmPosition() ;

	//get the Homogenous transformation End Effector of the left arm
	//Used in XR1Controller , Needs to be refreshed every simulation step when used
	//Argu: N/A
	//Reutrns : A homogenous transformation (4x4)
	MatrixXd getRightArmPositionMatrix() ;


	//Trun ON/OFF the tilt compensation, only affects Control groups in Position Mode
	//Used in XR1Controller.
	//Argu: option true/false, if true, it will calculate an offfset value based on the orientation of the OmniWheels
	//Reutrns : void , may add error message in the fulture
	void tiltCompensateSwitch(bool option);


	//Trun ON/OFF the dynamic compensation options of Left/Right Arm, only affects the internal calculation
	//Used in XR1Controller.
	//Argu: option true/false, if true, it will calculate an offfset value based on the orientation of the OmniWheels
	//Reutrns : void , may add error message in the fulture
	void setInverseDynamicsOption(uint8_t option);


	std::map<uint8_t, uint8_t> ControlModes;


	// Convert Joint ROS Messages
	xr1controllerros::ArmMsgs ConvertArmMsgs(std::vector<double> input);

	xr1controllerros::ArmMsgs ConvertArmMsgs(Eigen::VectorXd input);

	xr1controllerros::BodyMsgs ConvertBodyMsgs(std::vector<double> input) ;

	xr1controllerros::BodyMsgs ConvertBodyMsgs(Eigen::VectorXd input);

	Eigen::VectorXd BodyMsgs2VectorXd(const xr1controllerros::BodyMsgs& msg);

	Eigen::VectorXd ArmMsgs2VectorXd(const xr1controllerros::ArmMsgs& msg);

	Eigen::VectorXd HandsMsgs2VectorXd(const xr1controllerros::HandMsgs& msg);

	geometry_msgs::Twist ConvertIkMsgs(const VectorXd& twist);

	geometry_msgs::Twist ConvertIkMsgs(const Vector3d& Linear , const Vector3d& Angular);

	geometry_msgs::Twist ConvertIkMsgs(std::vector<double> Linear , std::vector<double> Angular);

	xr1controllerros::HandMsgs ConvertHandMsgs(std::vector<double> HandPosition);

	xr1controllerros::HandMsgs ConvertHandMsgs(Eigen::VectorXd HandPosition);

protected:

	//Helper Functions that users should not worry about
	void updatingCallback(VectorXd JointValue, uint8_t control_group , uint8_t values_type);

	void tiltCompensate(const geometry_msgs::Quaternion& msg);

	void subscribeMainBodyPosition(const xr1controllerros::BodyMsgs& msg);

	void subscribeLeftArmPosition(const xr1controllerros::ArmMsgs& msg);

	void subscribeLeftArmVelocity(const xr1controllerros::ArmMsgs& msg);

	void subscribeLeftArmCurrent(const xr1controllerros::ArmMsgs& msg);

	void subscribeRightArmPosition(const xr1controllerros::ArmMsgs& msg);

	void subscribeRightArmVelocity(const xr1controllerros::ArmMsgs& msg);

	void subscribeRightArmCurrent(const xr1controllerros::ArmMsgs& msg);

	void subscribeLeftHandPosition(const xr1controllerros::HandMsgs& msg);

	void subscribeRightHandPosition(const xr1controllerros::HandMsgs& msg);

	void onZero();

	void simulationTimeCallback(const std_msgs::Float32& msg);

	void updateControllers(uint8_t ControlGroup , uint8_t InformationType);

	std::vector<uint8_t> ArrayIDHelper(uint8_t control_group);

	void PlaybackCallback();

	bool CollisionDection();

	Vector3d TiltCalcualtion(Matrix3d BaseRotation);

	Vector3d UnitVector2XY(Vector3d v);

	Vector3d Matrix2XY(Matrix3d BaseRotation);


private:

	// Pay no Attention Here Plz
	ros::NodeHandle nh;

	XR1ControllerPM * XR1_ptr;

	ros::Publisher SimulationStartPublisher;

	ros::Publisher SimulationPausePublisher;

	ros::Publisher SimulationStopPublisher;

	ros::Publisher MainBodyModeChangePublisher ;

	ros::Publisher LeftArmModeChangePublisher  ;

	ros::Publisher RightArmModeChangePublisher ;

	ros::Publisher LeftHandModeChangePublisher   ;

	ros::Publisher RightHandModeChangePublisher  ;


	ros::Publisher IKTargetPositionPublisher;

	ros::Publisher LeftHandPositionPublisher;

	ros::Publisher RightHandPositionPublisher;

	ros::Publisher JointVisualizationPublisher;

	ros::Publisher TwistPublisher;

	ros::Publisher LeftArmPositionPublisher;

	ros::Publisher RightArmPositionPublisher;

	ros::Publisher MainBodyPositionPublisher;

	ros::Publisher LeftArmVelocityPublisher;

	ros::Publisher RightArmVelocityPublisher;

	ros::Publisher LeftArmCurrentPublisher ;

	ros::Publisher RightArmCurrentPublisher;

	ros::Publisher LeftHandCurrentPublisher;

	ros::Publisher RightHandCurrentPublisher;

	ros::Publisher PlatformPublisher;

	ros::Publisher LeftArmEEFPositionPublisher   ;

	ros::Publisher RightArmEEFPositionPublisher  ;

	ros::Publisher SyncModePublisher;

	ros::Publisher TriggerPublisher;

	ros::Publisher JointAttributePublisher;

	ros::Subscriber MainBodyPositionSubscriber;

	ros::Subscriber LeftArmPositionSubscriber ;

	ros::Subscriber RightArmPositionSubscriber;

	ros::Subscriber LeftArmVelocitySubscriber;

	ros::Subscriber LeftArmCurrentSubscriber;

	ros::Subscriber RightArmVelocitySubscriber;

	ros::Subscriber RightArmCurrentSubscriber;

	ros::Subscriber simulationTimeSubscriber;

	ros::Subscriber JointCurrentPositionSubscriber;

	ros::Subscriber BaseRotSubscriber;

	ros::Publisher LeftHandMotionPlanningMethodPublisher;

	ros::Publisher RightHandMotionPlanningMethodPublisher;

	ros::Publisher LeftHandMotionPlanningTriggerPublisher;

	ros::Publisher RightHandMotionPlanningTriggerPublisher;

	double simulationTime;

	double previous_simulationTime;



}; //class

#endif // my_namespace__my_plugin_H
