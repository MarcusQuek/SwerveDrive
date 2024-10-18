#include <cstdint>
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include <cmath>
#include <sstream>
#include <vector>
#include "api.h"

// #define LEFT_UPPER_BEVEL_MOTOR_1 15
// #define LEFT_UPPER_BEVEL_MOTOR_2 12 
// #define LEFT_LOWER_BEVEL_MOTOR_1 1
// #define LEFT_LOWER_BEVEL_MOTOR_2 2
// #define RIGHT_UPPER_BEVEL_MOTOR_1 20
// #define RIGHT_UPPER_BEVEL_MOTOR_2 19
// #define RIGHT_LOWER_BEVEL_MOTOR_1 9
// #define RIGHT_LOWER_BEVEL_MOTOR_2 7

// #define LEFT_UPPER_BEVEL_MOTOR_1 11
// #define LEFT_UPPER_BEVEL_MOTOR_2 12
// #define LEFT_LOWER_BEVEL_MOTOR_1 19
// #define LEFT_LOWER_BEVEL_MOTOR_2 20
// #define RIGHT_UPPER_BEVEL_MOTOR_1 1
// #define RIGHT_UPPER_BEVEL_MOTOR_2 2
// #define RIGHT_LOWER_BEVEL_MOTOR_1 9
// #define RIGHT_LOWER_BEVEL_MOTOR_2 10

// #define LEFT_LIFT_MOTOR 18
// #define RIGHT_LIFT_MOTOR 8

// #define UPPER_INTAKE_MOTOR 15
// #define LOWER_INTAKE_MOTOR 14

// #define IMU_SENSOR_PORT 16
// #define SERIALPORT 17

// #define LEFT_ROTATION_SENSOR_PORT 13
// #define RIGHT_ROTATION_SENSOR_PORT 3

#define LEFT_UPPER_BEVEL_MOTOR_1 11
#define LEFT_UPPER_BEVEL_MOTOR_2 12
#define LEFT_LOWER_BEVEL_MOTOR_1 19
#define LEFT_LOWER_BEVEL_MOTOR_2 20
#define RIGHT_UPPER_BEVEL_MOTOR_1 2
#define RIGHT_UPPER_BEVEL_MOTOR_2 3
#define RIGHT_LOWER_BEVEL_MOTOR_1 8
#define RIGHT_LOWER_BEVEL_MOTOR_2 9

#define LEFT_LIFT_MOTOR 18
#define RIGHT_LIFT_MOTOR 10

#define UPPER_INTAKE_MOTOR 15
#define LOWER_INTAKE_MOTOR 14

#define IMU_SENSOR_PORT 17
#define SERIALPORT 16

#define LEFT_ROTATION_SENSOR_PORT 13
#define RIGHT_ROTATION_SENSOR_PORT 1

#define POTENTIOMETER_SENSOR_PORT 'H'
#define SOLENOID_SENSOR_PORT 'G'

#define ZERO_VECTOR INFINITY


pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor luA(LEFT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor luB(LEFT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llA(LEFT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llB(LEFT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor ruA(RIGHT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor ruB(RIGHT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlA(RIGHT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlB(RIGHT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor intakeLower(UPPER_INTAKE_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intakeUpper(LOWER_INTAKE_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor liftL(LEFT_LIFT_MOTOR, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor liftR(RIGHT_LIFT_MOTOR, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Rotation left_rotation_sensor(LEFT_ROTATION_SENSOR_PORT, true);
pros::Rotation right_rotation_sensor(RIGHT_ROTATION_SENSOR_PORT, true);
pros::Imu imu(IMU_SENSOR_PORT);
pros::ADIAnalogIn lifter(POTENTIOMETER_SENSOR_PORT);
pros::ADIDigitalOut solenoid(SOLENOID_SENSOR_PORT);

extern "C" int32_t vexGenericSerialReceive( uint32_t index, uint8_t *buffer, int32_t length );
extern "C" void vexGenericSerialEnable(  uint32_t index, uint32_t nu );
extern "C" void vexGenericSerialBaudrate(  uint32_t index, uint32_t rate );
extern "C" int32_t vexGenericSerialTransmit( uint32_t index, uint8_t *buffer, int32_t length );

//Controllers
int leftX = 0, leftY = 0, rightX = 0;
const double DEADBAND = 8.0;
const double MAX_RPM = 600.0;
const double SCALING_FACTOR = MAX_RPM / 127.0;
const double TO_DEGREES = (180.0 / M_PI);
const double TO_RADIANS = (M_PI / 180.0);
const double WHEEL_BASE_RADIUS = 0.2;

//SwerveTranslation
double left_wheel_speed = 0.0;
double right_wheel_speed = 0.0;

//SetWheelAngle
bool setAngle = false;

const double lkP = 2.1;
const double lkI = 0.0;
const double lkD = 0.0;

const double rkP = 2.1;
const double rkI = 0.0;
const double rkD = 0.0;

double target_angle = 1.5;
double target_angleL = 1.5;
double target_angleR = 1.5;
double left_turn_speed = 0.0;
double right_turn_speed = 0.0;
double rotational = 0.0;
double rotationalL = 0.0;
double rotationalR = 0.0;

//MogoLift
const double mkP = 0.88;
const double mkI = 0.0; 
const double mkD = 1.79;

int liftTarget = 0;
bool liftEnable = false;

bool isLeftFlipped = false;
bool isRightFlipped = false;

//Serial
double global_distY = 0.0;
double global_distX = 0.0;
double global_errorY = 0.0;
double global_errorX = 0.0;