#ifndef XR1CONTROLLERPM_H
#define XR1CONTROLLERPM_H


#include "genericcontroller.h"
#include "Eigen/Dense"
#include "chaincontroller.h"
#include "handcontroller.h"
#include "omnicontroller.h"
#include "dynamicmethod.h"
#include <map>
#include <vector>
#include "Eigen/Geometry"
#include "xr1define.h"




class XR1ControllerPM
{

public:
    XR1ControllerPM(string parameters_path);


    //Simple Joint Controls--------------------------------------

    //Set the Target Joint Positions for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    VectorXd setJointPosition(u_int8_t control_group ,VectorXd JA);

    //Set the Target Joint Velocity for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in Eigen::VectorXd
    //Reutrns : void , may add error message in the fulture
    VectorXd setJointVelocity(u_int8_t control_group ,VectorXd JV);

    //Set the Target Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Target Current
    //Reutrns : void , may add error message in the fulture
    VectorXd setJointCurrent(u_int8_t control_group ,VectorXd JC);


    //Set the Target Joint Positions for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angles contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointPosition(u_int8_t JointID ,double JA);

    //Set the Target Joint Velocity for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointVelocity(u_int8_t JointID ,double JV);


    //Set the Target Joint Acceleration for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointAcceleration(u_int8_t JointID ,double JACC);

    //Set the Target Joint Currents for a single joint, i.e. LeftShoulderX , RightWristZ
    //Used in the XR1Controller
    //Argu: Control Group ID , Angular Velocity contained in std::vector<double>
    //Reutrns : void , may add error message in the fulture
    void setJointCurrent(u_int8_t JointID ,double JC);

    //Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : JointAngles contained in Eigen::VectorXd
    VectorXd getJointPositions(u_int8_t control_group);

    //Get the Current Joint Angles for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : JointAngles contained in std::vector
    std::vector<double> getJointPositionsStd(u_int8_t control_group);

    //Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Velocities contained in Eigen::VectorXd
    VectorXd getJointVelocities(u_int8_t control_group);

    //Get the Current Joint Velocites for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Velocites contained in std::vector
    std::vector<double> getJointVelocitiesStd(u_int8_t control_group);

    //Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Currents contained in Eigen::VectorXd
    VectorXd getJointCurrents(u_int8_t control_group);

    //Get the Current Joint Currents for an entire control group, i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID
    //Reutrns : Joint Currents contained in std::vector
    std::vector<double> getJointCurrentsStd(u_int8_t control_group);


    //For each joint
    double getJointAngle(u_int8_t joint_id);

    double getJointVelocity(u_int8_t joint_id);

    double getJointCurrent(u_int8_t joint_id);

    //---------------------------------------------------------------------------------
    //Get Target Position for Arms or Body
    VectorXd getTargetPosition(u_int8_t control_group);

    //Get Target velocity for Arms or Body
    VectorXd getTargetVelocity(u_int8_t control_group);

    //Get Target Current for Arms or Body
    VectorXd getTargetCurrent(u_int8_t control_group);

    double getTargetJointPosition(u_int8_t joint_id);

    double getTargetJointVelocity(u_int8_t joint_id);

    double getTargetJointCurrent(u_int8_t joint_id);


    //Zero all the values
    void Zero();

    //-----------------------------------------------------------------


    //Commnunications--------------------------------------------------
    //Set the Control Method for an entire Control Group , i.e. LeftARM , RightHand
    //Used in the XR1Controller
    //Argu: Control Group ID , Conrol Mode ID
    //Reutrns : void , may add error message in the fulture
    void setControlMode(u_int8_t control_group ,u_int8_t option);
    u_int8_t getControlMode(u_int8_t control_group);


    void errorHandle();


    //Update the actual values for controllers
    void updatingCallback(VectorXd JointValue, u_int8_t control_group , u_int8_t values_type);
    void updatingCallback(double JointValue, u_int8_t JointID , u_int8_t values_type);


    // Trigger the Calcualtions
    //Subjected to change
    //Argu: N/A
    //Reutrns : void , may add error message in the fulture
    void triggerCalculation();

    //--------------------------------------------------------------------------------

    //Toggle the dynamic modes for left and right arm, be very careful with the options
    void setInverseDynamicsOption(u_int8_t control_group , u_int8_t option);
    //----------------------------------------------------------------------------------



    MatrixXd getJacobian(u_int8_t joint_idx);

    std::vector<MatrixXd> getJacobian(std::vector<u_int8_t> joint_idx_list);


    // Brutal Straight Forward Controls----------------------------------------------
    void setEndEffectorIncrement(u_int8_t control_group ,const Vector3d& Linear , const Vector3d& Angular);
    void setLeftArmIncrement(const Vector3d& Linear , const Vector3d& Angular);
    void setRightArmIncrement(const Vector3d& Linear , const Vector3d& Angular);

    void setEndEffectorVelocity(u_int8_t control_group ,const Vector3d& Linear , const Vector3d& Angular);
    void setLeftArmVelocity(const Vector3d& Linear , const Vector3d& Angular);
    void setRightArmVelocity(const Vector3d& Linear , const Vector3d& Angular);

    void setEndEffectorForce(u_int8_t control_group , const Vector3d& Linear , const Vector3d& Torque);
    void setLeftArmForce(const Vector3d& Force , const Vector3d& Torque);
    void setRightArmForce(const Vector3d& Force , const Vector3d& Torque);

    void setLeftArmIncrement(const VectorXd& twist);
    void setRightArmIncrement(const VectorXd& twist);

    void setLeftArmVelocity(const VectorXd& twist);
    void setRightArmVelocity(const VectorXd& twist);

    void setLeftArmForce(const VectorXd& twist);
    void setRightArmForce(const VectorXd& twist);


    VectorXd getLeftArmForce();

    VectorXd getLeftArmVelocity();

    VectorXd getLeftArmPosition();

    MatrixXd getLeftArmPositionMatrix();

    VectorXd getRightArmForce();

    VectorXd getRightArmVelocity();

    VectorXd getRightArmPosition();

    MatrixXd getRightArmPositionMatrix();



    //-----------------------------------------------------------------

    Vector3d TiltCompensation(Matrix3d BaseRotation, Vector3d BaseAcceleration);

    Vector3d TiltCompensation(Quaterniond BaseRotation , Vector3d BaseAcceleration);



    //Dynamics Controls-----------------------------------------------------------------

    void updateBaseTransformation();


    double tinyBezier(double double_index , double pt_s , double pt_1 , double pt_2 , double pt_e);

    bool CollisionDetection(u_int8_t control_group);

    void setPeriod(u_int8_t control_group, double reading_interval_in_second);


    //OmniWheels Controls-----------------------------------------------------------------
    void SetOmniWheelsVelocity(Vector3d input);


    void setState(std::vector<double> goal_configuration ,double  period, double control_rate = 200);

    std::vector<double> getNextState();

private:



    virtual void tinyBezier(double & path_p , double & start_p, double & first_p, double & last_p, double & end_p, double double_index);

    virtual void tinyBezvel(double & path_v , double & start_p, double & first_p, double & last_p, double & end_p, double  double_index);

    virtual void tinyBezacc(double & path_a , double & start_p, double & first_p, double & last_p, double & end_p, double  double_index);


    virtual void getState();

    virtual void tinyBezier(std::vector<double> & path , double & start_p, double & first_p, double & last_p, double & end_p, double & time_s , double rate = 200);

    virtual void tinyBezier(std::vector<double> & path ,double & start_p, double & end_p, double & time_s ,double rate = 200);

    virtual void groupBezier(std::vector<std::vector<double> > & path , std::vector<double> & start_p, std::vector<double> & end_p, double & time_s ,double rate = 200);



    std::map<u_int8_t ,GenericController *> ControllerMap;

    std::map<u_int8_t , u_int8_t> ControllerIDs;

    ChainController * LeftArm;

    ChainController * RightArm;

    ChainController * MainBody;


    HandController * LeftHand;

    HandController * RightHand;

    OmniController * OmniWheels;

    DynamicMethod * DungeonMaster;


    std::map<u_int8_t, u_int8_t> ControlModes;



    std::vector<std::vector<double> > GeneratedCommands;

    Vector3d TiltOffset;

    int PlaybackIndex;


    void readParameters(string parameters_path);

    std::vector<std::vector<double> > readParameter(std::string parameter_path);

    std::vector<u_int8_t> ArrayIDHelper(u_int8_t control_group);

    void PlaybackCallback();

    bool GripDetection(u_int8_t joint_id);

    bool CollisionThresholding(VectorXd ActualCurrent , VectorXd ExpectedCurrent, std::vector<double> Thresholds, std::vector<double> StaticThreshold);

    bool ReleaseThresholding(VectorXd ActualPosition, VectorXd TargetPosition, VectorXd Thresholds);

    bool CollisionThresholding(double ActualCurrent , double ExpectedCurrent, double Threshold);

    bool ReleaseThresholding(double ActualPosition, double TargetPosition, double Threshold);


    Vector3d TiltCalcualtion(Matrix3d BaseRotation, Vector3d BaseAcceleration);

    Vector3d Matrix2XY(Matrix3d BaseRotation);

    Matrix3d EulerZX(double z , double x);

    Matrix3d XY2Matrix(Vector3d BaseRotation);

    Vector3d Vector2XY(Vector3d BaseVector);

    int num_joint_in_chain;

    int num_joint_in_hand;

    int num_joint_friction;

    int num_joint_parameters;


    double grip_current;

    std::vector<double> start_state;
    std::vector<double> goal_state;
    std::vector<double> current_state;

    double bezier_period;
    double bezier_index;
    double bezier_rate;

    std::vector<double> LeftArmCollisionThreshold;

    std::vector<double> RightArmCollisionThreshold;

    std::vector<double> LeftArmStaticThreshold ;

    std::vector<double> RightArmStaticThreshold;

    double LeftArmCollisionCounter;

    double RightArmCollisionCounter;


    double LeftArmCollisionCounterLimit;

    double RightArmCollisionCounterLimit;

    double HandCollisionThreshold;


};

#endif // XR1CONTROLLERPM_H
