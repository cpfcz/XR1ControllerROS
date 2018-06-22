#ifndef XR1CONTROLLERPM_H
#define XR1CONTROLLERPM_H

#include "xr1controllerpm_global.h"
#include "Eigen/Dense"
#include "chaincontroller.h"
#include "handcontroller.h"
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


    //

    VectorXd getJointPositions(u_int8_t control_group);

    std::vector<double> getJointPositionsStd(u_int8_t control_group);

    VectorXd getJointVelocities(u_int8_t control_group);

    std::vector<double> getJointVelocitiesStd(u_int8_t control_group);

    VectorXd getJointCurrents(u_int8_t control_group);

    std::vector<double> getJointCurrentsStd(u_int8_t control_group);

    void Zero();

    //-----------------------------------------------------------------


    //Commnunications--------------------------------------------------

    void setControlMode(u_int8_t control_group ,u_int8_t option);

    void errorHandle();

    void launchingCallback();

    void updatingCallback(VectorXd JointValue, u_int8_t control_group , u_int8_t values_type);

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


    Vector3d TiltCompensation(Matrix3d BaseRotation);

    Vector3d TiltCompensation(Quaterniond BaseRotation);

    //---------------------------------------------------------------------------------

    VectorXd getTargetPosition(u_int8_t control_group);

    VectorXd getTargetVelocity(u_int8_t control_group);

    VectorXd getTargetCurrent(u_int8_t control_group);

    //Dynamics Controls-----------------------------------------------------------------

    void updateBaseTransformation();



private:
    ChainController * LeftArm;

    ChainController * RightArm;

    ChainController * MainBody;


    HandController * LeftHand;

    HandController * RightHand;

    VectorXd Joint_Angles;

    VectorXd Joint_Velocities;

    VectorXd Joint_Currents;

    std::vector<std::vector<double> > GeneratedCommands;

    Vector3d TiltOffset;

    int PlaybackIndex;


    void updatingControllers();

    void readingCallback();

    std::vector<u_int8_t> ArrayIDHelper(u_int8_t control_group);

    void PlaybackCallback();

    bool CollisionDection();

    bool CollisionThresholding(VectorXd ActualCurrent , VectorXd ExpectedCurrent);

    Vector3d TiltCalcualtion(Matrix3d BaseRotation);

    Vector3d UnitVector2XY(Vector3d v);

    Vector3d Matrix2XY(Matrix3d BaseRotation);

    int num_joint_in_chain;
};

#endif // XR1CONTROLLERPM_H
