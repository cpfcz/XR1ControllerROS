#ifndef HANDCONTROLLER_H
#define HANDCONTROLLER_H

#include "Eigen/Dense"
#include "xr1define.h"
#include <vector>
using namespace Eigen;
class HandController
{
public:
    HandController(u_int8_t id);

    void updateValue(VectorXd JointValue , u_int8_t value_type);

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


private:
    VectorXd Joint_Angles;
    VectorXd Joint_Velocities;
    VectorXd Joint_Currents;
    VectorXd Target_Joint_Angles;
    VectorXd Target_Joint_Velocities;
    VectorXd Target_Joint_Currents;
    u_int8_t Begin_ID;
    int NUM_OF_JOINTS;
};

#endif // HANDCONTROLLER_H
