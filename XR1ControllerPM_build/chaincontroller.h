#ifndef CHAINCONTROLLER_H
#define CHAINCONTROLLER_H

#include "genericcontroller.h"
#include "Eigen/Dense"
#include "xr1define.h"
#include <vector>

using namespace Eigen;

class ChainController: public GenericController
{
public:
    ChainController(MatrixXd DH_input, u_int8_t id , VectorXd Inertia_Parameters , VectorXd Gravity_Parameters, int num_joint);

    VectorXd getTargetJointCurrents();

    std::vector<double> getTargetJointCurrentsStd();

    //For each joint

    double getTargetJointCurrent(u_int8_t joint_id);


    VectorXd getEFFForce();

    VectorXd getEFFVelocity();

    VectorXd getEFFPosition();

    MatrixXd getEFFPositionMatrix();

    Vector3d Matrix2XYZ(Matrix3d BaseRotation);

    // Update the the base
    void updateBaseTransformation(Matrix3d BaseT);

    // Returns the last calculated Jacobian matrix
    MatrixXd getJacobian(u_int8_t id);

    // Return the last calculated end effector Transformation
    MatrixXd getTransformation(u_int8_t JointID);



    void triggerCalculationPass();

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

    VectorXd InertiaParameters4dof;

    VectorXd InertiaParameters7dof;


    // Saves Jacobian matrix as a the member variable
    void Jacobeans(VectorXd Joint_Angles);


    // Saves individual transformation in the transformation collection
    void Transformation(VectorXd Joint_Angles);
    MatrixXd T_DH(double d , double offset , double alpha , double ad , double theta);

    MatrixXd BigDee(int idx_start, int idx_end);

    Matrix3d EulerXYZ(double x , double y , double z ) ;

    Matrix3d EulerZYX(double x , double y , double z ) ;

    MatrixXd dothat(Vector3d v);

    MatrixXd pinv(MatrixXd input);

    Matrix3d BaseRotationTransposed;


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


    //Intermediate for Newton Euler
    std::vector<MatrixXd> Ks ;






    //Not Intermediate for Newton Euler;
    double t2  ;
    double t3  ;
    double t4  ;
    double t5  ;
    double t6  ;
    double t7  ;
    double t8  ;
    double t9  ;
    double t10 ;
    double t11 ;
    double t12 ;
    double t13 ;
    double t14 ;
    double t15 ;
    double t16 ;
    double t17 ;
    double t18 ;
    double t19 ;
    double t20 ;
    double t21 ;
    double t22 ;



    double gx ;
    double gy ;
    double gz ;


    MatrixXd Regressor;



    // For Transforms
    double costheta;
    double sintheta;
    double sinalpha;
    double cosalpha;

    // Jacobeans
    Vector3d v;
    Vector3d w_j;
    Vector3d z;

    Vector3d temp_v_1;
    Vector3d temp_v_2;


    //Transformation

    MatrixXd Trans;

    MatrixXd Temp_Trans;




};

#endif // CHAINCONTROLLER_H
