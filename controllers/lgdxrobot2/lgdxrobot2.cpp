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
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include <iostream>

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
  std::cout << "IK: W1: " << wheels[0]->getVelocity() << " W2: " << wheels[1]->getVelocity() << " W3: " << wheels[2]->getVelocity() << " W4: " << wheels[0]->getVelocity() << std::endl;
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
  
  PositionSensor *positionSensors[4];
  char position_name[4][9] = {"encoder1", "encoder2", "encoder3", "encoder4"};
  for (int i = 0; i < 4; i++) {
    positionSensors[i] = robot->getPositionSensor(position_name[i]);
    positionSensors[i]->enable(timeStep);
  }
  
  motorSetIk(wheels, 0.1, 0, 0);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  double motorFk[3] = {0};
  double motorPositionChange[4] = {0};
  double motorPosition[4] = {0};
  double motorLastPosition[4] = {0};
  double lastTime = 0;
  while (robot->step(timeStep) != -1) {
  double curTime = robot->getTime();
  double dt = curTime - lastTime;
    for (int i = 0; i < 4; i++) {
      motorPosition[i] = positionSensors[i]->getValue();
      motorPositionChange[i] = motorPosition[i] - motorLastPosition[i];
    }
    //std::cout << "E1: " << motorPositionChange[0] << " E2: " << motorPositionChange[1] << " E3: " << motorPositionChange[2] << " E4: " << motorPositionChange[3]<< std::endl;
    motorFk[0] = ((motorPositionChange[0] + motorPositionChange[1] + motorPositionChange[2] + motorPositionChange[3]) * (WHEEL_RADIUS / 4)) / dt;
    motorFk[1] = ((-motorPositionChange[0] + motorPositionChange[1] + motorPositionChange[2] - motorPositionChange[3]) * (WHEEL_RADIUS / 4)) / dt;
    motorFk[2] = ((-motorPositionChange[0] + motorPositionChange[1] - motorPositionChange[2] + motorPositionChange[3]) * ((WHEEL_RADIUS) / (4 * (CHASSIS_LX + CHASSIS_LY)))) / dt;
    //std::cout << " X: " << motorFk[0] << " Y: " << motorFk[1] << " W: " << motorFk[2] << std::endl;
    for (int i = 0; i < 4; i++) {
      motorLastPosition[i] = motorPosition[i];
    }
    lastTime = curTime;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
