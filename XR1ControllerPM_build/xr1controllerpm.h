#ifndef XR1CONTROLLERPM_H
#define XR1CONTROLLERPM_H

#include "xr1controllerpm_global.h"
#include "genericcontroller.h"
#include "Eigen/Dense"
#include "chaincontroller.h"
#include "handcontroller.h"
#include "omnicontroller.h"
#include <map>
#include <vector>
#include "Eigen/Geometry"
#include "xr1define.h"




class XR1CONTROLLERPMSHARED_EXPORT XR1ControllerPM
{

public:
    XR1ControllerPM();


    //Simple Joint Controls--------------------------------------
    VectorXd setJointPosition(u_int8_t control_group ,VectorXd JA);

    VectorXd setJointVelocity(u_int8_t control_group ,VectorXd JV);

    VectorXd setJointCurrent(u_int8_t control_group ,VectorXd JC);



    void setJointPosition(u_int8_t JointID ,double JA);

    void setJointVelocity(u_int8_t JointID ,double JV);

    void setJointCurrent(u_int8_t JointID ,double JC);

    //

    VectorXd getJointPositions(u_int8_t control_group);

    std::vector<double> getJointPositionsStd(u_int8_t control_group);

    VectorXd getJointVelocities(u_int8_t control_group);

    std::vector<double> getJointVelocitiesStd(u_int8_t control_group);

    VectorXd getJointCurrents(u_int8_t control_group);

    std::vector<double> getJointCurrentsStd(u_int8_t control_group);


    //For each joint
    double getJointAngle(u_int8_t joint_id);

    double getJointVelocity(u_int8_t joint_id);

    double getJointCurrent(u_int8_t joint_id);


    void Zero();

    //-----------------------------------------------------------------


    //Commnunications--------------------------------------------------

    void setControlMode(u_int8_t control_group ,u_int8_t option);
    u_int8_t getControlMode(u_int8_t control_group);

    void errorHandle();

    void launchingCallback();

    void updatingCallback(VectorXd JointValue, u_int8_t control_group , u_int8_t values_type);
    void updatingCallback(double JointValue, u_int8_t JointID , u_int8_t values_type);

    //-----------------------------------------------------------------

    MatrixXd getJacobian(u_int8_t joint_idx);

    std::vector<MatrixXd> getJacobian(std::vector<u_int8_t> joint_idx_list);


    // Brutal Straight Forward Controls----------------------------------------------
    void setLeftArmIncrement(const Vector3d& Linear , const Vector3d& Angular);

    void setRightArmIncrement(const Vector3d& Linear , const Vector3d& Angular);

    void setLeftArmVelocity(const Vector3d& Linear , const Vector3d& Angular);

    void setRightArmVelocity(const Vector3d& Linear , const Vector3d& Angular);

    void setLeftArmCurrent(const Vector3d& Force , const Vector3d& Torque);

    void setRightArmCurrent(const Vector3d& Force , const Vector3d& Torque);

    void setLeftArmIncrement(const VectorXd& twist);

    void setRightArmIncrement(const VectorXd& twist);

    void setLeftArmVelocity(const VectorXd& twist);

    void setRightArmVelocity(const VectorXd& twist);

    void setLeftArmCurrent(const VectorXd& twist);

    void setRightArmCurrent(const VectorXd& twist);

    VectorXd getLeftArmForce();

    VectorXd getLeftArmVelocity();

    VectorXd getLeftArmPosition();

    MatrixXd getLeftArmPositionMatrix();

    VectorXd getRightArmForce();

    VectorXd getRightArmVelocity();

    VectorXd getRightArmPosition();

    MatrixXd getRightArmPositionMatrix();

    //--------------------------------------------------------------------------------

    void setLeftArmDynamicSwitch(bool option);

    void setRightArmDynamicSwitch(bool option);

    //----------------------------------------------------------------------------------
    void playGeneratedData();

    void readGeneratedData(std::vector<std::vector<double> > input);

    void clearGeneratedData();


    Vector3d TiltCompensation(Matrix3d BaseRotation, Vector3d BaseAcceleration);

    Vector3d TiltCompensation(Quaterniond BaseRotation , Vector3d BaseAcceleration);

    //---------------------------------------------------------------------------------

    VectorXd getTargetPosition(u_int8_t control_group);

    VectorXd getTargetVelocity(u_int8_t control_group);

    VectorXd getTargetCurrent(u_int8_t control_group);

    double getTargetJointPosition(u_int8_t joint_id);

    double getTargetJointVelocity(u_int8_t joint_id);

    double getTargetJointCurrent(u_int8_t joint_id);

    //Dynamics Controls-----------------------------------------------------------------

    void updateBaseTransformation();

    void triggerCalculation();

    double tinyBezier(double double_index , double pt_s , double pt_1 , double pt_2 , double pt_e);

    bool CollisionDetection(u_int8_t control_group);


    //OmniWheels Controls-----------------------------------------------------------------
    void SetOmniWheelsVelocity(Vector3d input);



private:

    std::map<u_int8_t ,GenericController *> ControllerMap;

    std::map<u_int8_t , u_int8_t> ControllerIDs;

    ChainController * LeftArm;

    ChainController * RightArm;

    ChainController * MainBody;


    HandController * LeftHand;

    HandController * RightHand;

    OmniController * OmniWheels;

    std::map<u_int8_t, u_int8_t> ControlModes;



    VectorXd Joint_Angles;

    VectorXd Joint_Velocities;

    VectorXd Joint_Currents;

    std::vector<std::vector<double> > GeneratedCommands;

    Vector3d TiltOffset;

    int PlaybackIndex;


    std::vector<u_int8_t> ArrayIDHelper(u_int8_t control_group);

    void PlaybackCallback();

    bool GripDetection(u_int8_t joint_id);

    bool CollisionThresholding(VectorXd ActualCurrent , VectorXd ExpectedCurrent, VectorXd Thresholds);

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

    double grip_current;

    VectorXd LeftArmCollisionThreshold;

    VectorXd RightArmCollisionThreshold;

    double HandCollisionThreshold;


};

#endif // XR1CONTROLLERPM_H
