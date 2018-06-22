#ifndef CHAINCONTROLLER_H
#define CHAINCONTROLLER_H

#include "Eigen/Dense"
#include "xr1define.h"
#include <vector>

using namespace Eigen;

class ChainController
{
public:
    ChainController(MatrixXd DH_input, u_int8_t id , VectorXd Inertia_Parameters);



    VectorXd getEFFForce();

    VectorXd getEFFVelocity();

    VectorXd getEFFPosition();

    MatrixXd getEFFPositionMatrix();

    Vector3d Matrix2XYZ(Matrix3d BaseRotation);

    // Update the the base
    void updateBaseTransformation(MatrixXd BaseT);

    // Returns the last calculated Jacobian matrix
    MatrixXd getJacobian(u_int8_t id);

    // Return the last calculated end effector Transformation
    MatrixXd getTransformation(u_int8_t JointID);

    VectorXd getJointAngles();

    std::vector<double> getJointAnglesStd();

    VectorXd getJointVelocities();

    std::vector<double> getJointVelocitiesStd();

    VectorXd getJointCurrents();

    std::vector<double> getJointCurrentsStd();

    VectorXd getTargetJointAngles();

    std::vector<double> getTargetJointAnglesStd();

    VectorXd getTargetJointVelocities();

    std::vector<double> getTargetJointVelocitiesStd();

    VectorXd getTargetJointCurrents();

    std::vector<double> getTargetJointCurrentsStd();

    void updateValue(VectorXd JointValue , u_int8_t value_type);

    VectorXd NewtonEuler();





    void setEFFIncrement(const Vector3d& Linear , const Vector3d& Angular);

    void setEFFVelocity(const Vector3d& Linear , const Vector3d& Angular);

    void setEFFCurrent(const Vector3d& Force , const Vector3d& Torque);

    void setEFFIncrement(const VectorXd& twist);

    void setEFFVelocity(const VectorXd& twist);

    void setEFFCurrent(const VectorXd& twist);

    bool DynamicSwitch;

private:

    MatrixXd DH_parameters;

    double shoulder_angle_offset;

    double la1;
    double la2;
    double la3;
    double la4;
    double la5;

    std::vector<MatrixXd> Jacobians;

    std::vector<MatrixXd> Transformations_Collection;
    std::vector<MatrixXd> Trans_Collection;

    VectorXd InertiaParameters;

    const int NUM_OF_JOINTS;

    //Newton Euler Equations for computing the inverse dynamics
    //Use this for dynamics


    std::vector<VectorXd> IANewtonEuler();

    // Saves Jacobian matrix as a the member variable
    void Jacobeans(VectorXd Joint_Angles);


    // Saves individual transformation in the transformation collection
    void Transformation(VectorXd Joint_Angles);
    MatrixXd T_DH(double d , double offset , double alpha , double ad , double theta);


    //Transform 3d vector into Skew Matrix
    Matrix3d Hatty_Hattington(Vector3d input);


    //Transform wenches with Adjoint matrices
    MatrixXd BigDee(int idx_start , int idx_end);


    Matrix3d EulerXYZ(double x , double y , double z ) ;

    Matrix3d EulerZYX(double x , double y , double z ) ;

    MatrixXd dothat(Vector3d v);

    MatrixXd pinv(MatrixXd input);

    Matrix3d BaseRotationTransposed;

    VectorXd Joint_Angles;

    VectorXd Joint_Velocities;

    VectorXd Joint_Currents;

    VectorXd Joint_Acceleration;

    VectorXd Target_Joint_Angles;

    VectorXd Target_Joint_Velocities;

    VectorXd Target_Joint_Acceleration;

    VectorXd Target_Joint_Currents;

    VectorXd Dynamic_Compensation;

    VectorXd d ;
    VectorXd ad ;
    VectorXd alpha ;
    VectorXd offset;


    int NUM_PARA;


    u_int8_t Begin_ID;

    double g;
    double PI;

    Vector3d grav;



protected:

    double S1;   double C1;
    double S2;   double C2;
    double S3;   double C3;
    double S4;   double C4;
    double S5;   double C5;
    double S6;   double C6;
    double S7;   double C7;

};

#endif // CHAINCONTROLLER_H
