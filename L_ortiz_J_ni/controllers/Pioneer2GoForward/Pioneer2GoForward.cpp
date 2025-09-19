// Include dependencies 
#include <webots/Robot.hpp> 
#include <webots/Motor.hpp>

// Define time step increment used by Webots to advance  
// the virtual time and perform physics simulation.
#define TIME_STEP 64 

// Webots classes use "webots" namespace 
using namespace webots;

// Main function of the controller   
int main(int argc, char **argv) { 
// Initialize robot instance   
  Robot *robot = new Robot(); 

// Get Pioneer II motors 
  Motor *leftMotor = robot->getMotor("left wheel motor"); 
  Motor *rightMotor = robot->getMotor("right wheel motor"); 

// Assign a position to both motors 
  leftMotor->setPosition(INFINITY); 
  rightMotor->setPosition(INFINITY);
  
  // Set motor's velocity
  leftMotor->setVelocity(5);
  rightMotor->setVelocity(5);
  
  while (robot->step(TIME_STEP) != -1);

// Delete robot instance  
  delete robot; 
  return 0;
}