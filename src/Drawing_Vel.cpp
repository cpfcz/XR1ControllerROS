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



// The trajectory for position
Eigen::Vector3d  TrajectoryPositionFunction(const double& t) {

  Eigen::Vector3d res(0, 0, 0);

  res(0) = 0.4 ;
  res(1) = 0.0425 * sin(2.0 * t) + 0.0425 * cos(t) - 0.0425;
  res(2) = 0.0375 - 0.0375 * sin(t) - 0.0375 * cos(2.0 * t);

  return res;
}

// The same trajectory for velocity
Eigen::Vector3d  TrajectorySpeedFunction(const double& t ) {


  Eigen::Vector3d res(0, 0, 0);

  res(0) = 0;
  res(1) = 0.085 * cos(2.0 * t) - 0.0425 * sin(t);
  res(2) = 0.075 * sin(2.0 * t) - 0.0375 * cos(t);

  return res;
}

// The same trajectory for accelration
Eigen::Vector3d  TrajectoryAccelrationFunction(const double& t ) {


  Eigen::Vector3d res(0, 0, 0);

  res(0) = 0;
  res(1) = - 0.17 * sin(2.0 * t) - 0.0425 * cos(t);
  res(2) = 0.15 * cos(2.0 * t) + 0.0375 * sin(t);

  return res;
}


// Called when each simulation step is finished
void stepFinishedCallback(const std_msgs::Bool& msg) {

  static double t;

  static Vector3d pos_error_gain = Eigen::VectorXd::Zero(3) ;

  pos_error_gain << 5.0 , 5.0 , 5.0;


  static Vector3d rot_error_gain = Eigen::VectorXd::Zero(3) ;

  rot_error_gain <<  0.5, 0.5, 0.5;



  t = XR1_ptr->getSimulationTime() - simualtionStartTime ;


  if (!ready) {

    if (XR1_ptr->ControlModes[XR1::RightArm ] != XR1::IKMode)
      XR1_ptr->setControlMode(XR1::RightArm , XR1::IKMode);

    if (XR1_ptr->ControlModes[XR1::LeftArm ] != XR1::IKMode)
      XR1_ptr->setControlMode(XR1::LeftArm , XR1::IKMode);



    //Uses Inverse Kinematics Mode to get into position
    static Eigen::VectorXd startPOS = Eigen::VectorXd::Zero(6);

    startPOS << 0.4 , 0 , 0 , -3.14 / 2.0 , 3.14 , 0;

    if (t <= 2.0) {

      XR1_ptr->setLeftArmPosition( (startPOS - InitPOSL) / 2.0 * t + InitPOSL);

      XR1_ptr->setRightArmPosition( (startPOS - InitPOSR) / 2.0 * t + InitPOSR);
    }


    if (t  > 3.0) {

      ready = true;

      simualtionStartTime = XR1_ptr->getSimulationTime();


      // Changing Mode to Start Drawing

      ROS_INFO("Start Drawing");

      XR1_ptr->setControlMode(XR1::RightArm , XR1::VelocityMode);

      XR1_ptr->setControlMode(XR1::LeftArm , XR1::VelocityMode);

      XR1_ptr->setInverseDynamicsOption(XR1::RightArm , XR1::GravityCompensation);


    }

  }

  else {


    //The Loop For drawing loops (Haha)

    XR1_ptr->setLeftArmVelocity(TrajectorySpeedFunction(t) , Eigen::VectorXd::Zero(3)) ;

    //The Right Arm is commanded with Velocity Mode
    XR1_ptr->setLeftArmVelocity(TrajectorySpeedFunction(t) , Eigen::VectorXd::Zero(3))  ;

  }

  //This is the command to finalize all the commnuication and trigger the next step
  XR1_ptr->stepFinishedCallback();
}



//
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


  //V-REP will lag for a good second before it is ready to receive ROS messages
  ros::Duration(1).sleep();


  //The simulation Time is being updated as soon as it is running
  simualtionStartTime = XR1_ptr->getSimulationTime();


  //Get the Initial End Effector transormation
  InitPOSL = XR1_ptr->getLeftArmPosition();
  InitPOSR = XR1_ptr->getRightArmPosition();

  // Some of the caluclations are done after the simulation step
  StepFinishedSubscriber = nh.subscribe("/simulationStepDone", 1 , &stepFinishedCallback);


  // Trigger the Next step, it is important to run this command once after the  subscriber is set,
  // since the first message can be lost
  XR1_ptr->stepFinishedCallback();

  ros::spin();
}