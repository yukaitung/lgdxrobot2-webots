// File:          lgdxrobot2.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define CHASSIS_LX 0.237
#define CHASSIS_LY 0.287
#define WHEEL_RADIUS 0.0375

// All the webots classes are defined in the "webots" namespace
using namespace webots;

void motorSetIk(Motor **wheels, float velocityX, float velocityY, float velocityW)
{
  wheels[0]->setVelocity((1 / WHEEL_RADIUS) * (velocityX - velocityY - (CHASSIS_LX + CHASSIS_LY) * velocityW));
  wheels[1]->setVelocity((1 / WHEEL_RADIUS) * (velocityX + velocityY + (CHASSIS_LX + CHASSIS_LY) * velocityW));
  wheels[2]->setVelocity((1 / WHEEL_RADIUS) * (velocityX + velocityY - (CHASSIS_LX + CHASSIS_LY) * velocityW));
  wheels[3]->setVelocity((1 / WHEEL_RADIUS) * (velocityX - velocityY + (CHASSIS_LX + CHASSIS_LY) * velocityW));
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Camera *camera = robot->getCamera("camera rgb");
  camera->enable(timeStep);
  
  RangeFinder *rangeFinder = robot->getRangeFinder("camera depth");
  rangeFinder->enable(timeStep);
  
  Lidar *lidar = robot->getLidar("lidar");
  lidar->enable(timeStep);
  lidar->enablePointCloud();
  
  // get lidar motor and enable rotation (only for visualization, no effect on the sensor)
  Motor *lidarMainMotor = robot->getMotor("lidar_main_motor");
  Motor *lidarSecondaryMotor = robot->getMotor("lidar_secondary_motor");
  lidarMainMotor->setPosition(INFINITY);
  lidarSecondaryMotor->setPosition(INFINITY);
  lidarMainMotor->setVelocity(30);
  lidarSecondaryMotor->setVelocity(60);
  
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
  }
  
  motorSetIk(wheels, 0, 0, 0.5);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);

  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
