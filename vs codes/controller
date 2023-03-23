#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#define TIME_STEP 64
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Keyboard kb;
  DistanceSensor *ds[5];
  
  char dsNames[5][20] = {"ds front right", "ds front left","ds side right", "ds side left", "ds upper"};
  for (int i = 0; i < 5; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  
  
  //updown motor
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
  
  // GPS *gp;
  // gp=robot->getGPS("global");
  // gp-> enable(TIME_STEP);
  
  // InertialUnit *iu;
  // iu=robot->getInertialUnit("imu");
  // iu-> enable(TIME_STEP);


  
  Motor *wheels[4];
  char wheels_names[4][18] = {"front_left_wheel", "front_right_wheel", "left_back_wheel", "right_back_wheel"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  kb.enable(TIME_STEP);
  double leftSpeed = 0.0;
  double rightSpeed = 0.0;
  double linear=0.0;
  
  double linear2=0.0;
  
  while (robot->step(TIME_STEP) != -1) {
    int key=kb.getKey();
    //std::cout<<key<<std::endl;
    
    if (key==315){
    leftSpeed = 5.0;
    rightSpeed = 5.0;
    } else if (key==317){
    leftSpeed = -5.0;
    rightSpeed = -5.0;
    }else if (key==316){
    leftSpeed = 5.0;
    rightSpeed = -5.0;
    }else if (key==314){
    leftSpeed = -5.0;
    rightSpeed = 5.0;
    }else {
    leftSpeed = 0.0;
    rightSpeed = 0.0;
    }
     //std::cout<<ds[0]->getValue()<<"=Right Sensor"<<std::endl;
    // std::cout<<ds[1]->getValue()<<"=Left Sensor"<<std::endl;
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    wheels[2]->setVelocity(leftSpeed);
    wheels[3]->setVelocity(rightSpeed);
    // std::cout<<"X : "<<gp->getValues()[0]<<std::endl;
    // std::cout<<"Y : "<<gp->getValues()[1]<<std::endl;
    // std::cout<<"Z : "<<gp->getValues()[2]<<std::endl;
    // std::cout<<"########################"<<std::endl;
    // std::cout<<"Angle X : "<<iu->getRollPitchYaw()[0]<<std::endl;
    // std::cout<<"Angle Y : "<<iu->getRollPitchYaw()[1]<<std::endl;
    // std::cout<<"Angle Z : "<<iu->getRollPitchYaw()[2]<<std::endl;
    if (key==87 && linear<0.19){
    linear += 0.005;
    } else if (key==83 && linear>0){
    linear += -0.005;
    }else {
    linear +=0;
    }
    lr->setPosition(linear);
    
    if (key==65 && linear2<0.19){
    linear2 += 0.005;
    } else if (key==68 && linear2>0){
    linear2 += -0.005;
    }else {
    linear2 +=0;
    }
    la->setPosition(linear2);
    ra->setPosition(linear2);
    fa->setPosition(linear2);
    ba->setPosition(linear2);
  
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}