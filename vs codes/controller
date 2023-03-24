

// File:          EPuckAvoideCollision.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28
// All the webots classes are defined in the "webots" namespace
using namespace webots;

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
    // initialize devices
  DistanceSensor *ps[7];
  char psNames[7][20] = {
    "ds side right1","ds side right2" ,"ds front left", "ds front right", "ds side left1","ds side left2",
    "ds upper"
  };
  for (int i = 0; i < 7; i++) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
  }
  
  
  Motor *lr;
  lr=robot->getMotor("linear_motor");
 
   Motor *la;
  la=robot->getMotor("left_arm"); 
 
    Motor *ra;
  ra=robot->getMotor("right_arm"); 
  
    Motor *fa;
  fa=robot->getMotor("front_arm");
  
    Motor *ba;
  ba=robot->getMotor("back_arm");
  
    
  Motor *wheels[4];
  char wheels_names[4][18] = {"front_left_wheel", "front_right_wheel", "left_back_wheel", "right_back_wheel"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
 
  
  
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
  // detect obstacles
  double psValues[5];
  for (int i = 0; i < 5 ; i++)
  psValues[i] = ps[i]->getValue();
      
    bool right_obstacle =
      psValues[0] < 800.0 ;
      
    bool left_obstacle =
      psValues[3] < 800.0;
      
    bool front_obstacle =
      psValues[1] < 800.0||
      psValues[2] < 800.0;
      std::cout<<psValues[1]<<"=front Sensor1"<<std::endl;
      std::cout<<psValues[2]<<"=front Sensor2"<<std::endl;
      std::cout<<psValues[0]<<"=right Sensor2"<<std::endl;
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.
    // initialize motor speeds at 50% of MAX_SPEED.
    double leftSpeed  = 0.4 * MAX_SPEED;
    double rightSpeed = 0.4 * MAX_SPEED;
    // modify speeds according to obstacles
    if (front_obstacle) {
      // turn right
      leftSpeed  = 0 * MAX_SPEED;
      rightSpeed = 0 * MAX_SPEED;
      wheels[0]->setVelocity(leftSpeed);
      wheels[1]->setVelocity(rightSpeed);
      wheels[2]->setVelocity(leftSpeed);
      wheels[3]->setVelocity(rightSpeed);
      std::cout<<"front obstacle"<<std::endl;
      
      if (!right_obstacle) {
      // turn right
      leftSpeed  = 0.4 * MAX_SPEED;
      rightSpeed = -0.4 * MAX_SPEED;
      wheels[0]->setVelocity(leftSpeed);
      wheels[1]->setVelocity(rightSpeed);
      wheels[2]->setVelocity(leftSpeed);
      wheels[3]->setVelocity(rightSpeed);
      
      std::cout<<" no right obstacle"<<std::endl;
      }
      else if (!left_obstacle) {
      // turn right
      leftSpeed  = -0.4 * MAX_SPEED;
      rightSpeed = 0.4 * MAX_SPEED;
      wheels[0]->setVelocity(leftSpeed);
      wheels[1]->setVelocity(rightSpeed);
      wheels[2]->setVelocity(leftSpeed);
      wheels[3]->setVelocity(rightSpeed);
      
      std::cout<<" no left obstacle"<<std::endl;
      }
    }
    
    else {
      // forward
      leftSpeed  += 0.4 * MAX_SPEED;
      rightSpeed += 0.4 * MAX_SPEED;
    }
    // write actuators inputs
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    wheels[2]->setVelocity(leftSpeed);
    wheels[3]->setVelocity(rightSpeed);
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
