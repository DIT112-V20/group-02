#include <Smartcar.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include <VL53L0X.h>

BluetoothSerial bluetooth;

// Constansts
const float SPEED = 0.8;            // Standard Speed in m/s
const int RIGHT = 90;               // 90 Degrees to turn on spot right
const int LEFT = -90;               // 90 Degrees to turn on spot left
const int SIDE_MIN_OBSTACLE = 22;   // Minimum distance for SR04
const int FRONT_MIN_OBSTACLE = 150; // Minimum distance for Micro-LIDAR
const int GYROSCOPE_OFFSET = 13;
const unsigned int MAX_DISTANCE = 100; // Max distance to measure with ultrasonic
const float SPEEDCHANGE = 0.1;         // Used when increasing and decreasing speed. Might need a new more concrete name?
// Autodrive direction variables
const int ORIENTATION_FORWARD = 0;
const int ORIENTATION_RIGHT = 1;
const int ORIENTATION_BACK = 2;
const int ORIENTATION_LEFT = 3;

// Variables
float currentSpeed;

// Unsigned
unsigned int sideSensorError = 3;   //Errormargin for side sensors
unsigned int frontSensorError = 30; //Errormargin for front ensor
unsigned int frontDistance;
unsigned int leftDistance;
unsigned int rightDistance;

// Boolean
boolean atObstacleFront = false;
boolean atObstacleLeft = false;
boolean atObstacleRight = false;
boolean autoDrivingEnabled = false;

// Ultrasonic trigger pins
const int TRIGGER_PIN = 5; // Trigger signal
const int ECHO_PIN = 18;   // Reads signal
const int TRIGGER_PIN1 = 19;
const int ECHO_PIN1 = 23;

// Sensor pins
SR04 leftSensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Ultrasonic measures in centimeters
SR04 rightSensor(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
VL53L0X frontSensor;         // Micro LIDAR measures in millimeters
GY50 gyro(GYROSCOPE_OFFSET); // Gyroscope

// Odometer
const unsigned long PULSES_PER_METER = 600; // TODO CALIBRATE PULSES ON CAR
DirectionlessOdometer leftOdometer(
    smartcarlib::pins::v2::leftOdometerPin, []() { leftOdometer.update(); }, PULSES_PER_METER);
DirectionlessOdometer rightOdometer(
    smartcarlib::pins::v2::rightOdometerPin, []() { rightOdometer.update(); }, PULSES_PER_METER);

// Car motors
BrushedMotor leftMotor(smartcarlib::pins::v2::leftMotorPins);
BrushedMotor rightMotor(smartcarlib::pins::v2::rightMotorPins);
DifferentialControl control(leftMotor, rightMotor);

// Car initializing
SmartCar car(control, gyro, leftOdometer, rightOdometer);

void setup()
{
    car.enableCruiseControl(); // Enables Cruisecontrol in order to work with m/s
    Serial.begin(9600);
    Wire.begin();
    frontSensor.setTimeout(500);
    if (!frontSensor.init())
    {
        Serial.print("Failed to initialize VL53L0X sensor.");
        while (1)
        {
        }
    }
    frontSensor.startContinuous(100);
    bluetooth.begin("Group 2 SmartCar");
}

// Bluetooth inputs
void readBluetooth()
{
    while (bluetooth.available())
    {
        char input = bluetooth.read();
        manualControl(input); // Sends input to  the manualcontrol to control the car
    }
}

// Bluetooth outputs
void writeBluetooth(byte message)
{
    if (bluetooth.hasClient())
    {
        bluetooth.write(&message, 1);
    }
}

// Rotate on spot function for the automaticDriving
void rotateOnSpot(int targetDegrees)
{
    car.disableCruiseControl(); // Disables cruiseControl in order to use OverrideMotorSpeed
    int speed = 40;
    targetDegrees %= 360; // Puts it on a (-360,360) scale

    if (!targetDegrees)
    {
        return;
    }

    // Sets overrideMotorSpeed values depending on targetDegrees
    if (targetDegrees > 0)
    {
        car.overrideMotorSpeed(speed, -speed);
    }
    else
    {
        car.overrideMotorSpeed(-speed, speed);
    }

    const auto initialHeading = car.getHeading();
    int degreesTurnedSoFar = 0;

    while (abs(degreesTurnedSoFar) < abs(targetDegrees))
    {
        car.update();
        auto currentHeading = car.getHeading();

        if ((targetDegrees < 0) && (currentHeading > initialHeading))
        {
            // Turning left while current heading is bigger than the initial one
            currentHeading -= 360;
        }
        else if ((targetDegrees > 0) && (currentHeading < initialHeading))
        {
            // Turning right while current heading is smaller than the initial one
            currentHeading += 360;
        }

        degreesTurnedSoFar = initialHeading - currentHeading;
    }
    car.setSpeed(0);
    car.enableCruiseControl(); // ReEnable cruiseControl
}

// Car rotate for manualControl
void rotate(int degrees, float speed)
{
    degrees %= 360; // Put degrees in a (-360,360) scale
    car.setSpeed(speed);

    if (speed < 0)
    { // Checks if we are driving backward or forward and sets angle accordingly
        if (degrees > 0)
        {
            car.setAngle(LEFT);
        }
        else
        {
            car.setAngle(RIGHT);
        }
    }
    else
    {
        if (degrees > 0)
        {
            car.setAngle(RIGHT);
        }
        else
        {
            car.setAngle(LEFT);
        }
    }
    const auto initialHeading = car.getHeading();
    bool hasReachedTargetDegrees = false;
    while (!hasReachedTargetDegrees)
    { // While car hasn't turned enough
        car.update();
        auto currentHeading = car.getHeading();
        if (degrees < 0 && currentHeading > initialHeading)
        { // Turning while current heading is bigger than the initial one
            currentHeading -= 360;
        }
        else if (degrees > 0 && currentHeading < initialHeading)
        { // Turning while current heading is smaller than the initial one
            currentHeading += 360;
        }

        int degreesTurnedSoFar = initialHeading - currentHeading;
        hasReachedTargetDegrees = smartcarlib::utils::getAbsolute(degreesTurnedSoFar) >= smartcarlib::utils::getAbsolute(degrees);
    }
    car.setSpeed(0);
}

// Manual forward drive
void driveForward()
{
    car.setAngle(0);
    car.update();
    float currentSpeed = car.getSpeed();
    // Gradually increase the car speed in order to not overwork the motors.
    while (currentSpeed < SPEED)
    {
        car.setSpeed(currentSpeed += SPEEDCHANGE);
    }
}

// Manual backwards drive
void driveBackward()
{
    car.setAngle(0);
    car.update();
    float currentSpeed = car.getSpeed();
    // The car will gradually increase the backward speed to reduce motor overloading.
    while (currentSpeed > -SPEED)
    {
        car.setSpeed(currentSpeed -= SPEEDCHANGE);
    }
}

// Carstop
void stopCar()
{
    car.setSpeed(0);
}

// Not yet used. Will most likely not be used. Here for testing.
void driveDistance(long distance, float speed)
{
    long initialDistance = car.getDistance();
    long travelledDistance = 0;

    if (speed > 0)
    {
        driveForward();
    }
    else
    {
        driveBackward();
    }

    while (travelledDistance <= distance)
    {
        car.update();
        long currentDistance = car.getDistance();
        travelledDistance = currentDistance - initialDistance;
    }
    stopCar();
}

// Obstacle interference forward
void checkFrontObstacle()
{
    frontDistance = (frontSensor.readRangeSingleMillimeters() - frontSensorError);
    if (frontSensor.timeoutOccurred())
    {
        Serial.print("VL53L0X sensor timeout occurred.");
    }
    atObstacleFront = (frontDistance > 0 && frontDistance <= FRONT_MIN_OBSTACLE) ? true : false;
}

// Obstacle interference left
void checkLeftObstacle()
{
    leftDistance = leftSensor.getDistance();
    atObstacleLeft = (leftDistance > 0 && leftDistance <= SIDE_MIN_OBSTACLE) ? true : false;
}

// Obstacle interference right
void checkRightObstacle()
{
    rightDistance = rightSensor.getDistance();
    atObstacleRight = (rightDistance > 0 && rightDistance <= SIDE_MIN_OBSTACLE) ? true : false;
}

// Automated driving with obstacle avoidance
void automatedDriving()
{
    while (autoDrivingEnabled)
    {
        checkFrontObstacle();
        // Drive forward until there is an obstacle in front of car
        while (!atObstacleFront)
        {
            driveForward();
            writeBluetooth('f');
            checkFrontObstacle();
        }
        checkLeftObstacle();
        checkRightObstacle();

        avoidObstacle(0, ORIENTATION_RIGHT); // Initialise obstacle avoidance
    }
}

void avoidObstacle(int totalDistance, int turns)
{
    /* Implementing the "Wall Follower" algorithm for maze solving.
       Always have the obstacle to the left of the car. If we return to the
       straight line the car was on before encountering the obstacle, we are clear
        of the obstacle. */

    float currentDistance = 0;
    float tempDistance;

    // Always have the wall to the left of the car
    while (!atObstacleFront && atObstacleLeft)
    {
        if (abs(currentDistance) < 0.1 && turns % 4 == ORIENTATION_LEFT)
        { // If we are driving back left and get close to distance needed, exit method
            rotateOnSpot(RIGHT);
            writeBluetooth('r');
            return;
        }
        tempDistance = car.getDistance();
        driveForward();
        writeBluetooth('f');
        checkFrontObstacle();
        checkLeftObstacle();
        currentDistance += calculateDistance(tempDistance, turns);
    }
    totalDistance += currentDistance;

    checkRightObstacle();

    if (!atObstacleLeft)
    { // If left is clear, rotate and iterate obstacle avoidance again
        rotateOnSpot(LEFT);
        writeBluetooth('l');
        avoidObstacle(totalDistance, turns - 1);
    }
    else if (!atObstacleRight)
    { // If right is clear, rotate and iterate obstacle avoidance again
        rotateOnSpot(RIGHT);
        writeBluetooth('r');
        avoidObstacle(totalDistance, turns + 1);
    }
    else if (atObstacleFront)
    { // If all sides and front is hindered, drive backwards
        tempDistance = car.getDistance();

        while (atObstacleRight)
        {
            driveBackward();
            writeBluetooth('b');
            checkRightObstacle();
        }
        totalDistance -= calculateDistance(tempDistance, turns);

        rotateOnSpot(RIGHT);
        writeBluetooth('r');
        avoidObstacle(totalDistance, turns + 1);
    }
}

float calculateDistance(float tempDistance, int turns)
{
    // Returns the distance travelled along the x axis. Positive if the car is travelling East
    // Negative if the car is travelling to the west
    float currentDistance;
    // Calculate the distance traversed in this iteration
    if (turns % 4 == ORIENTATION_RIGHT)
    {
        currentDistance = car.getDistance() - tempDistance;
    }
    else if (turns % 4 == ORIENTATION_LEFT)
    {
        currentDistance = -(car.getDistance() - tempDistance);
    }
    return currentDistance;
}

// Drive inputs
void manualControl(char input)
{
    switch (input)
    {

    case 'a': // Auto
        autoDrivingEnabled = true;
        automatedDriving();
        break;

    case 'm': // Manual
        autoDrivingEnabled = false;
        driveForward();
        break;

    case 'f': // Forward
        driveForward();
        break;

    case 'b': // Backwards
        driveBackward();
        break;

    case 'l': // Left turn
        rotate(LEFT, SPEED);
        break;

    case 'r': // Right turn
        rotate(RIGHT, SPEED);
        break;

    case 'k': // Left backwards turn
        rotate(LEFT, -SPEED);
        break;

    case 'j': // Right backwards turn
        rotate(RIGHT, -SPEED);
        break;

    case 'i': // Increases carspeed by 0.1. Not implemented in app yet
        car.setSpeed(car.getSpeed() + SPEEDCHANGE);
        break;

    case 'd': // Decreases carspeed by 0.1. Not implemented in app yet
        car.setSpeed(car.getSpeed() - SPEEDCHANGE);
        break;

    default:
        stopCar();
    }
}

void loop()
{
    readBluetooth();
    car.update();
}