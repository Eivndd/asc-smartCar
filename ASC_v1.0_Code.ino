/*
 * -- Authors: Magnus B Caro and Eivind T Haldorsen :)
 * 
 * 
 * In the report on page 10 to 30, we have documented our thoughtprocess
 * during the development of the system as well as explainations in
 * why we chose the different programming solutions. We also documented 
 * programingprogression in which we explain how we solved different programming problems.
*/

//Including different libraries, which includes library for the stepper motor, 
//Wire library to communicate with I2C devices and LiDAR library to use each LiDAR.
#include <CheapStepper.h>
#include <Wire.h>
#include <VL53L0X.h>

//Sensor setups and defines
#define XSHUT_pin1 3
#define XSHUT_pin2 5
#define XSHUT_pin3 4

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

//DC motor setup and defines
#define motorDriverIn1 7
#define motorDriverIn2 12
#define motorDriverEn 13

//Stepper motor setup and defines
CheapStepper stepper(48, 46, 44, 42);

//Button Switches
const int startAutoPin = 2;
const int stopAutoPin = 6;
const int resetWheelPosPin = 30;

//LED and Photoresistor
const int led1Pin = 50;
const int led2Pin = 38;
const int led3Pin = 36;
const int pResPin = 11;

//Variables
int tmpTurn = 0;            //Keeps track on the turn degree on the wheels
int hStop = 0;              //Variable which starts counting if the car is standing still
int maxturnrate = (45 + 1); //45 degree turnrate
bool moved = false;         //Controlls if the car is correcting wheels when reversing
bool disableLidar = false;  //Disable LiDARs when correcting wheels
bool startOn = false;       //Used to start and stop the autonomous car

void sensorSetup() {
  //Enable lidar sensor pins
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(3, LOW);

  delay(500);

  //SENSOR 1 ADDRESSING
  pinMode(5, INPUT);
  delay(150);
  sensor1.init(true);
  delay(100);
  sensor1.setAddress((uint8_t)22);

  //SENSOR 2 ADDRESSING
  pinMode(4, INPUT);
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)25);

  //SENSOR 3 ADDRESSING
  pinMode(3, INPUT);
  delay(150);
  sensor3.init(true);
  delay(100);
  sensor3.setAddress((uint8_t)28);

  //Start continuous mode to enable constant readings from the LiDARs
  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();

}

void dcMotorSetup() {
  //Enalbe DC motor driver pins
  pinMode(motorDriverEn, OUTPUT);
  pinMode(motorDriverIn1, OUTPUT);
  pinMode(motorDriverIn2, OUTPUT);
  digitalWrite(motorDriverEn, HIGH);
}

void setup()
{
  //Begin communication with I2C device
  Wire.begin();

  //Setup stepper motor pins
  pinMode(48, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(42, OUTPUT);

  //Setup led pins
  pinMode(led1Pin, INPUT);
  pinMode(led2Pin, INPUT);
  pinMode(led3Pin, OUTPUT);

  //Setup button pins
  pinMode(startAutoPin, INPUT);
  pinMode(stopAutoPin, INPUT);

  //Call the setups
  sensorSetup();
  dcMotorSetup();
}


//Method to measure and compare lengths between LiDAR A, and LiDAR B.
void compareLength(VL53L0X a, VL53L0X b) {
  int deadzone = 15; //15 mm deadzone;
  if (a.readRangeContinuousMillimeters() > (b.readRangeContinuousMillimeters() + uint16_t(deadzone))) { //Compares the sensors
    if (tmpTurn < maxturnrate) {        //If steering pin does not exceed its maximum turning degree.
      stepper.newMoveDegrees(true, 5);  //Turn stepper motor 5 degrees
      tmpTurn += 5;                     //Store in a variable that the stepper have moved 5 degrees
    }
  } else if ((a.readRangeContinuousMillimeters() + uint16_t(deadzone)) < b.readRangeContinuousMillimeters()) {
    if (tmpTurn > (-maxturnrate)) {     //Same here, just other direction
      stepper.newMoveDegrees(false, 5); //False = counter-clockwise
      tmpTurn -= 5;                     //Store that the stepper have moved
    }
  }
}


//Method to check the length of the center sensor
bool checkCentre() {
  int midRange = 450; //Center sensor length in millimetres
                      //Define the center lidar length before stopping initiates.
  if (sensor2.readRangeContinuousMillimeters() > uint16_t(midRange)) {
    return true;      //Stopping is required
  } else {
    return false;     //Stopping is not required
  }
}

//Method to straighten wheels, with parameter T in how much the wheels steer the wheels opposite.
void straightenCar(int t) {
  if (stepper.getStepsLeft() == 0) {                    //Checks wether the stepper motor runs or not, nothing runs if stepper motor is active
    if (tmpTurn < 0) {                                  //Checks which direction the wheels originaly are placed.
      stepper.newMoveDegrees(true, (t + abs(tmpTurn))); //Move the stepper in opposite direction + t with the variable tmpTurn that stored the previous wheel pos.
      tmpTurn = t;                                      //Set tmpTurn to t to store the new wheel position
      moved = true;                                     //Indicated that the car is correcting wheels, used to prioritize steering correction before reversing.
      return;
    } else if (tmpTurn > 0) {                           //Same concept, just opposite direction
      stepper.newMoveDegrees(false, (t + abs(tmpTurn)));
      tmpTurn = -t;                                     //Stores -t to tmpTurn for the new wheel position
      moved = true;
      return;
    }
  }
}

//Method to start a counter when the DC motor stops, hStop is the counter and is reset everytime the DC motor starts.
//This will allow us to start counting when the car meets a wall and initiates straightenCar and reverse whenever the counter reaches a specified time
void checkRev() {
  hStop += 1;             //Counter starts
  if (hStop > 75) {       //If counter reaches 75
    if (moved == false) { //Checks if the car have straightened the wheels
      straightenCar(25);  //Send in parameter 25 degrees from wheel center pos from opposite direction, which allows the car to turn AWAY from the obstacle
    } else {
      reverse();          //Initiates the reversing sequence after the wheels have been turned
    }
  }
}

//Method to reverse the car
void reverse() {                 
  uint32_t period = 2000L;                                           // 2 seconds
  for (uint32_t tStart = millis();  (millis() - tStart) < period;) { //For loop which uses the millis method to count the time up to "period" variable which is 2 seconds
    digitalWrite(motorDriverIn1, HIGH);                              //Send HIGH to In1 in motor driver
    digitalWrite(motorDriverIn2, LOW);                               //Send LOW to In2 in motor driver, these values will make the DC motor go the other way
  }
  hStop = 0;                                                         //Reset both the hStop variable and moved variable, 
  moved = false;                                                     //now the car have reversed and corrected wheels
}

//Simple method to check the photoresistorPIN for PWM signals.
//Based on these PWM signals, each LED will be enabled when the light is low.
void controlLights() {
  int value = analogRead(pResPin);      //Set value to photoresistor input data
  if ((value < 50) and (value >= 0)) {  //Checks the lower values
    digitalWrite(led1Pin, HIGH);
    digitalWrite(led2Pin, HIGH);
    digitalWrite(led3Pin, HIGH);
  }
  if ((value < 100) and (value >= 50)) { //Checks the low-mid values
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, HIGH);
    digitalWrite(led3Pin, HIGH);
  }
  if ((value < 170) and (value >= 100)) { //Checks the mid values
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, LOW);
    digitalWrite(led3Pin, HIGH);
  }
  if ((value < 255) and (value >= 170)) { //Checks the high values
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, LOW);
    digitalWrite(led3Pin, LOW);
  }
}

//Method that checks if any of the 3 buttons is pressed
void checkButtons() {    
  //Green button --> Starts autonomous driving after 2.5 seconds.                  
  if (digitalRead(startAutoPin) == HIGH) {
    delay(2500);
    startOn = true;
  }

  //Red button --> Disables autonomous driving
  if (digitalRead(stopAutoPin) == HIGH) {
    startOn = false;
  }

  //Yellow button --> Centers the steering pin.
  if (digitalRead(resetWheelPosPin) == HIGH) {
    straightenCar(0);
    moved = false;
  }
}


void loop() {
  checkButtons();                           //Constantly checks if a button is being pressed
  if (startOn == true) {                    //If green button have been pressed
    stepper.run();                          //Initiate the stepper with the run method from CheapStepper library
    if (stepper.getStepsLeft() == 0) {      //This IF will prioritize most of the proccesing power on the stepper, checks how many steps left in stepper
      if (hStop == 0) {                     //If the car is not reversing or correcting wheels
        compareLength(sensor1, sensor3);    //Compare sensor1 length(left pos on car) with sensor3 (right pos on car), and turn the car respectively
      }
      controlLights();                      //Run the control light method to enable the LEDs.
      if (checkCentre() == true) {          //Checks the center lidar
        digitalWrite(motorDriverIn1, LOW);  //In1 set to LOW
        digitalWrite(motorDriverIn2, HIGH); //In2 set to HIGH, sets DC motor to drive the car forward
        hStop = 0;                          //Everytime the car can drive, the counter for reversing is reset. Prevents the car from revering randomly
      } else {
        digitalWrite(motorDriverIn1, LOW);  //Stops the car if centre LiDAR sees object
        digitalWrite(motorDriverIn2, LOW);
        checkRev();                         //Initiate reverse counter which in turn will maybe initiate straightenCar and reversing methods
      }
    }
  } else {
    digitalWrite(motorDriverIn1, LOW);      //If red button is pressed, stop the car.
    digitalWrite(motorDriverIn2, LOW);
  }
}
