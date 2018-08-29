#ifndef GENERICCONTROLLER_H
#define GENERICCONTROLLER_H
#include "Eigen/Dense"
#include <vector>
#include "xr1define.h"

using namespace Eigen;
class GenericController
{
public:
    GenericController(u_int8_t id , int num_joint);

    virtual void updateValue(VectorXd JointValue , u_int8_t value_type);

    virtual void updateValue(double JointValue, u_int8_t JointID , u_int8_t value_type);

    //Each Group
    virtual VectorXd getJointAngles();

    virtual std::vector<double> getJointAnglesStd();

    virtual VectorXd getJointVelocities();

    virtual std::vector<double> getJointVelocitiesStd();

    virtual VectorXd getJointCurrents();

    virtual std::vector<double> getJointCurrentsStd();

    virtual VectorXd getTargetJointAngles();

    virtual std::vector<double> getTargetJointAnglesStd();

    virtual VectorXd getTargetJointVelocities();

    virtual std::vector<double> getTargetJointVelocitiesStd();

    virtual VectorXd getTargetJointCurrents();

    virtual std::vector<double> getTargetJointCurrentsStd();



    //For each finger
    virtual double getJointAngle(u_int8_t joint_id);

    virtual double getJointVelocity(u_int8_t joint_id);

    virtual double getJointCurrent(u_int8_t joint_id);

    virtual double getTargetJointAngle(u_int8_t joint_id);

    virtual double getTargetJointVelocity(u_int8_t joint_id);

    virtual double getTargetJointCurrent(u_int8_t joint_id);


    //Transform 3d vector into Skew Matrix
    virtual Matrix3d Hatty_Hattington(Vector3d input);

protected:
    VectorXd Joint_Angles;
    VectorXd Joint_Velocities;
    VectorXd Joint_Acceleration;
    VectorXd Joint_Currents;
    VectorXd Target_Joint_Angles;
    VectorXd Target_Joint_Velocities;
    VectorXd Target_Joint_Currents;
    u_int8_t Begin_ID;
    int NUM_OF_JOINTS;
    double period;

    double tinyFilter(double new_v , double old_v);

    double simpleFilter(double new_val , double input , double ratios);

    Vector3d simpleFilter(Vector3d new_val , Vector3d input , double ratios);
};

#endif // GENERICCONTROLLER_H
