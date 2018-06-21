#include <ros/ros.h>
#include "XR1ControllerROS.h"
#include "xr1define.h"
#include "std_msgs/Bool.h"
#include "Eigen/Dense"
#include <fstream>

XR1ControllerROS * XR1_ptr;

ros::Subscriber StepFinishedSubscriber;

int someCountingIdx;

double simualtionStartTime;

bool ready;

Eigen::VectorXd InitPOSL;
Eigen::VectorXd InitPOSR;

std::vector<std::vector<double> > stored_data;

bool wrote;


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

void writeData(){
    std::ofstream filename ("/home/rocky/CollectedData/testrun.txt");

  if (filename.is_open()){
    ROS_INFO("FileOpened");

    for (int i = 0 ; i < stored_data.size() ; i++) {

      for ( int j = 0 ; j < stored_data[i].size() ; j++) filename << stored_data[i][j] << "," ;

      filename << std::endl;
    }

  }
  else
    ROS_INFO("FileFailed");

  filename.close();

  wrote = true;
}

void stepFinishedCallback(const std_msgs::Bool& msg) {

  ROS_INFO("Enter callback [%f]" , XR1_ptr->getSimulationTime());

  static double t;

  t = XR1_ptr->getSimulationTime() - simualtionStartTime ;


  if (!ready) {

    static Eigen::VectorXd startPOS = Eigen::VectorXd::Zero(6);

    startPOS << 0.4 , 0 , 0 , 0 , 0 , 0;

    if (t <= 2.0) {
      ROS_INFO("SettingLeftArmPosition");
      XR1_ptr->setLeftArmPosition( (startPOS - InitPOSL) / 2.0 * t + InitPOSL);
      ROS_INFO("SettingRightArmPosition");
      XR1_ptr->setRightArmPosition( (startPOS - InitPOSR) / 2.0 * t + InitPOSR);
    }


    if (t  > 3.0) {

      ready = true;

      simualtionStartTime = XR1_ptr->getSimulationTime();

      XR1_ptr->setControlMode(XR1::Actuator_Total , XR1::VelocityMode);

      XR1_ptr->setControlMode(XR1::RightArm , XR1::ForceMode);
    }

  }

  else {
    
    XR1_ptr->setLeftArmVelocity(TrajectorySpeedFunction(t) , Eigen::VectorXd::Zero(3))  ;


    // static Eigen::VectorXd Error

    XR1_ptr->setRightArmVelocity(TrajectorySpeedFunction(t) , Eigen::VectorXd::Zero(3))  ;
    stored_data.push_back(XR1_ptr->getJointPositionsStd(XR1::LeftArm));
  }

  if(!wrote && t  > 7.0)
    writeData();


  ROS_INFO("step finishing");
  XR1_ptr->stepFinishedCallback();
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "Drawing_Vel");

  ros::NodeHandle nh;

  ready = false;

  wrote = false;

  XR1_ptr = new XR1ControllerROS();

  ROS_INFO("Started Controller");

  //The message will not be sent if it is excecuted right away
  ros::Duration(1).sleep();

  ROS_INFO("Launch");

  XR1_ptr->launchAllMotors(); // startSimulation()

  // ros::Duration(1).sleep();
  XR1_ptr->setControlMode(XR1::Actuator_Total , XR1::IKMode);

  // XR1_ptr->setControlMode(XR1::RightArm , XR1::IKMode);

  simualtionStartTime = XR1_ptr->getSimulationTime();

  ROS_INFO("Getting LeftArm Position");
  InitPOSL = XR1_ptr->getLeftArmPosition();
  ROS_INFO("Getting RightArm Position");
  InitPOSR = XR1_ptr->getRightArmPosition();

  StepFinishedSubscriber = nh.subscribe("/simulationStepDone", 1 , &stepFinishedCallback);

  XR1_ptr->stepFinishedCallback();

  ros::spin();
}