#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>

// time in [ms] of a simulation step
#define TIME_STEP 64

#define MIN_SPEED 2
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// entry point of the controller
int main(int argc, char **argv) {
  std::cout << "Running!\n";
  // create the Robot instance.
  Robot *robot = new Robot();

  // initialize devices
  DistanceSensor *ps[8];
  char psNames[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  // initialize devices
  LightSensor *ls[8];
  char lsNames[8][4] = {
    "ls0", "ls1", "ls2", "ls3",
    "ls4", "ls5", "ls6", "ls7"
  }; 
  
    for (int i = 0; i < 8; i++) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    ls[i] = robot->getLightSensor(lsNames[i]);
    ls[i]->enable(TIME_STEP);
    ps[i]->enable(TIME_STEP);
  }
  
  
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  double stuckCount = 0;

  // feedback loop: step simulation until an exit event is received
  while (robot->step(TIME_STEP) != -1) {
    // read sensors outputs
    double psValues[8];
    double lsValues[8];
    for (int i = 0; i < 8 ; i++) {
      lsValues[i] = ls[i]->getValue();
      psValues[i] = ps[i]->getValue();
    }
    // detect obstacles
    bool right_obstacle =
      psValues[0] > 80.0 ||
      psValues[1] > 80.0 ||
      psValues[2] > 80.0;
      
    bool left_obstacle =
      psValues[5] > 80.0 ||
      psValues[6] > 80.0 ||
      psValues[7] > 80.0;
      
    bool right_light =
      lsValues[0] < 3000 ||
      lsValues[1] < 3000 ||
      lsValues[2] < 3000 ||
      lsValues[3] < 3000;
    bool left_light =
      lsValues[4] < 3000 ||
      lsValues[5] < 3000 ||
      lsValues[6] < 3000 ||
      lsValues[7] < 3000;

    // initialize motor speeds at 50% of MAX_SPEED.
    double leftSpeed  = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;
    
    // check for obstacle & light, wiggle on out of there
    if (((left_obstacle || right_obstacle) && right_light) && stuckCount == 0) {
        std::cout << "Im Stuck!\n";
        stuckCount = 37;
    }
    else if (((left_obstacle || right_obstacle) && left_light) && stuckCount == 0) {
        
        stuckCount = 37;

    }
    if (stuckCount > 0) {
        leftSpeed = -MAX_SPEED;
        rightSpeed = MAX_SPEED;
        stuckCount--;
    }
    //If theres a light
    else if ((left_light || right_light) && stuckCount == 0) {
        if (lsValues[4] < 3000 && lsValues[3] < 3000) {
            leftSpeed = MAX_SPEED;
            rightSpeed = MAX_SPEED;
        }
        else if (right_light) {
       leftSpeed  = -0.5 * MAX_SPEED;
       rightSpeed = 0.5 * MAX_SPEED;
           std::cout<<lsValues[0] << "Right light sensor detected, turning!\n";
        
       }
    else if (left_light) {
      leftSpeed  = 0.5 * MAX_SPEED;
      rightSpeed = -0.5 * MAX_SPEED;
                 std::cout<<lsValues[7] << "Left light sensor detected, turning!\n";

      }
    }
    //no lights, check for obstacle
    else if ((left_obstacle) && stuckCount == 0) {
    
      // turn right
      leftSpeed  = 0.5 * MAX_SPEED;
      rightSpeed = -0.5 * MAX_SPEED;
      std::cout<<"Turning right!\n";
    }
    else if ((right_obstacle) && stuckCount == 0) {
      // turn left
      leftSpeed  = -0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
            std::cout<<"Turning Left!\n";
    }
        // write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
    
    for (int i = 0; i < 8 ; i++) {
      psValues[i] = ps[i]->getValue();
      lsValues[i] = ls[i]->getValue();
    }
  }

  delete robot;
  return 0; //EXIT_SUCCESS
}