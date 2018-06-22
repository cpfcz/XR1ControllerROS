#include <ros/ros.h>
#include "XR1ControllerROS.h"
#include "xr1define.h"
#include "std_msgs/Bool.h"
#include "Eigen/Dense"
#include <fstream>

XR1ControllerROS * XR1_ptr;

ros::Subscriber StepFinishedSubscriber;

double simualtionStartTime;

bool ready;

Eigen::VectorXd InitPOSL;

Eigen::VectorXd InitPOSR;



Eigen::Vector3d  TrajectoryPositionFunction(const double& t) {

  Eigen::Vector3d res(0, 0, 0);

  res(0) = 0.4 ;
  res(1) =0.0425*sin(2.0*t) + 0.0425*cos(t) - 0.0425;
  res(2) = 0.0375 - 0.0375*sin(t) - 0.0375*cos(2.0*t);

  return res;
}

Eigen::Vector3d  TrajectorySpeedFunction(const double& t ) {


  Eigen::Vector3d res(0, 0, 0);

  res(0) = 0;
  res(1) = 0.085*cos(2.0*t) - 0.0425*sin(t);
  res(2) = 0.075*sin(2.0*t) - 0.0375*cos(t);

  return res;
}

Eigen::Vector3d  TrajectoryAccelrationFunction(const double& t ) {


  Eigen::Vector3d res(0, 0, 0);

  res(0) = 0;
  res(1) = - 0.17*sin(2.0*t) - 0.0425*cos(t);
  res(2) = 0.15*cos(2.0*t) + 0.0375*sin(t);

  return res;
}


void stepFinishedCallback(const std_msgs::Bool& msg) {

  static double t;

  static VectorXd error_gain = Eigen::VectorXd::Zero(6) ;

  error_gain << 50.0 ,50.0 ,50.0 ,0.5,0.5,0.5;

  t = XR1_ptr->getSimulationTime() - simualtionStartTime ;


  if (!ready) {


    //Uses Inverse Kinematics Mode to get into position
    static Eigen::VectorXd startPOS = Eigen::VectorXd::Zero(6);

    startPOS << 0.4 , 0 , 0 , 0 , 0 , 0;

    if (t <= 2.0) {

      XR1_ptr->setLeftArmPosition( (startPOS - InitPOSL) / 2.0 * t + InitPOSL);

      XR1_ptr->setRightArmPosition( (startPOS - InitPOSR) / 2.0 * t + InitPOSR);
    }


    if (t  > 3.0) {

      ready = true;

      simualtionStartTime = XR1_ptr->getSimulationTime();


      // Changing Mode to Start Drawing 

      XR1_ptr->setControlMode(XR1::RightArm , XR1::VelocityMode);

      XR1_ptr->setControlMode(XR1::LeftArm , XR1::ForceMode);

      XR1_ptr->setLeftArmDynamicSwitch(true);

      
    }

  }

  else {
    

    //The Loop For Drawing 

    //LeftArm is commanded with Force Mode
    Eigen::VectorXd CurrentTwist = XR1_ptr->getLeftArmPosition();

    Eigen::VectorXd TargetTwist = Eigen::VectorXd::Zero(6);

    TargetTwist << TrajectoryPositionFunction(t) , 0 , 0 , 0;

    Eigen::VectorXd Errors = error_gain.cwiseProduct ((TargetTwist - CurrentTwist)  );

    XR1_ptr->setLeftArmCurrent( Errors ) ;

    XR1_ptr->setRightArmVelocity(TrajectorySpeedFunction(t) , Eigen::VectorXd::Zero(3))  ;


  }

  XR1_ptr->stepFinishedCallback();
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "Drawing_Vel");

  ros::NodeHandle nh;

  ready = false;

  XR1_ptr = new XR1ControllerROS();

  ROS_INFO("Started Controller");

  //The message will not be sent if it is excecuted right away
  ros::Duration(1).sleep();

  ROS_INFO("Launch");

  XR1_ptr->launchAllMotors(); // startSimulation()

  XR1_ptr->setControlMode(XR1::LeftArm , XR1::IKMode);

  XR1_ptr->setControlMode(XR1::RightArm , XR1::IKMode);

  simualtionStartTime = XR1_ptr->getSimulationTime();

  InitPOSL = XR1_ptr->getLeftArmPosition();

  InitPOSR = XR1_ptr->getRightArmPosition();

  StepFinishedSubscriber = nh.subscribe("/simulationStepDone", 1 , &stepFinishedCallback);

  XR1_ptr->stepFinishedCallback();

  ros::spin();
}