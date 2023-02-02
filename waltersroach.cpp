#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>

// time in [ms] of a simulation step
#define TIME_STEP 64

#define MIN_SPEED 2
#define MAX_SPEED 6.28

#define MAX_FOOD 500
#define EAT_SPEED 25

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// entry point of the controller
int main(int argc, char **argv) {
  std::cout << "Going to work!\n";  
  std::cout << "sanity check\n";

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
  
  DistanceSensor *ir0 = robot->getDistanceSensor("ir0");
  ir0->enable(TIME_STEP);
  LightSensor *cls0 = robot->getLightSensor("cls0");
  cls0->enable(TIME_STEP);
  LightSensor *cls1 = robot->getLightSensor("cls1");
  cls1->enable(TIME_STEP);
  LightSensor *cls2 = robot->getLightSensor("cls2");
  cls2->enable(TIME_STEP);
  LightSensor *cls3 = robot->getLightSensor("cls3");
  cls3->enable(TIME_STEP);
  

  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  double turnAroundCount = 0;

  bool previouslySmelled = false;
  bool previouslyInShadow = false;
  bool onFood = false;
  int hungerLevel = MAX_FOOD;

  // feedback loop: step simulation until an exit event is received
  while (robot->step(TIME_STEP) != -1) {
  
    // read sensors outputs
    double psValues[8];
    double lsValues[8];

    for (int i = 0; i < 8 ; i++) {
      lsValues[i] = ls[i]->getValue();
      psValues[i] = ps[i]->getValue();
    }
    double ir0_value = ir0->getValue();
    double cls0_value = cls0->getValue();
    double cls1_value = cls1->getValue();
    double cls2_value = cls2->getValue();
    double cls3_value = cls3->getValue();
    
    std::cout << cls0_value << "\t" << cls1_value << "\t" << cls2_value << "\t" << cls3_value << "\t";

    // detect obstacles
    bool right_obstacle =
      psValues[0] > 80.0 ||
      psValues[1] > 80.0 ||
      psValues[2] > 80.0;
      
    bool left_obstacle =
      psValues[5] > 80.0 ||
      psValues[6] > 80.0 ||
      psValues[7] > 80.0;
      
    bool left_shadow =
      cls0_value < 300 ||
      cls1_value < 300;
    bool right_shadow =
      cls2_value < 300 ||
      cls3_value < 300;
      
    bool left_light =
      cls0_value > 300 ||
      cls1_value > 300;
      
    bool right_light = 
      cls2_value > 300 ||
      cls3_value > 300;

    bool smell_food =
      ir0_value < 8.0;
    
    bool onFood = ir0_value < 5;

    // initialize motor speeds at 50% of MAX_SPEED.
    double leftSpeed  = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;

  // in a shadow look for food in shadow / stay in shadow
  if (left_shadow && right_shadow) {
    std::cout << "It's dark here!\n";
    previouslyInShadow = true;
  }

  // in sun or shadow, look for a shadow ASAP
  if (left_shadow || right_shadow) {
     if (left_shadow && !right_shadow) {
       //turn left
       std::cout<<"I'm partically in shade, turning left!\n";
       leftSpeed = -0.5 * MAX_SPEED;
       rightSpeed = 0.5 * MAX_SPEED;
     }
     else if (right_shadow && !left_shadow) {
              std::cout<<"I'm partically in shade, turning right!\n";
       rightSpeed = -0.5 * MAX_SPEED;
       leftSpeed = 0.5 * MAX_SPEED;
       
     }
     else if (right_shadow && left_light) {
         std::cout<<"I'm partically in shade, turning right!\n";
       rightSpeed = -0.5 * MAX_SPEED;
       leftSpeed = 0.5 * MAX_SPEED;
     }
     else if (left_shadow && right_light) {
         std::cout<<"I'm partically in shade, turning right!\n";
       rightSpeed = -0.5 * MAX_SPEED;
       leftSpeed = 0.5 * MAX_SPEED;
     }
     else if (!right_shadow && !left_shadow && previouslyInShadow) {
       turnAroundCount = 20;
       std::cout << "Leaving shadow, I should turn around!\n";
       previouslyInShadow = false;
     }
     
   }
   else {
     std::cout << "I need to find shade!\n";
   }
   
   //If robot is in shade, or there is no sun, go find food
   if (left_shadow || right_shadow) {

  //See if food is a smell of food
    if (smell_food) {
      std::cout << "Food detected!";
      previouslySmelled = true;
    }
    else if (previouslySmelled == true && (hungerLevel < (MAX_FOOD - 100))) {
      //No longer smelling food, turn around and go search more
      std::cout << "Food closeby, turning back!\n";
      turnAroundCount = 20;
      previouslySmelled = false;
    }
    else {
      previouslySmelled = false;
    }

  if (onFood) {
      if (hungerLevel < MAX_FOOD) {
        leftSpeed = 0;
        rightSpeed = 0;
        hungerLevel += EAT_SPEED;
        std::cout << hungerLevel << " Eating food\n";
      }
  }
  }

  // Regardless, we'll look for an obstacle
   if (left_obstacle) {
      // turn right
      leftSpeed  = 0.5 * MAX_SPEED;
      rightSpeed = -0.5 * MAX_SPEED;
      std::cout<<"Object on my left, turning right!\n";
    }
    else if (right_obstacle) {
      // turn left
      leftSpeed  = -0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
            std::cout<<"Object on my right, turning left!\n";
    }

  //Turn back around if leaving the scent of food
    if (turnAroundCount > 0) {
      std::cout << "Turning back!\n";
      leftSpeed = -0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
      turnAroundCount--;
    }
    hungerLevel--;
    // write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  }

  delete robot;
  return 0; //EXIT_SUCCESS
}