#include "main.h"

void disabled(){}
void competition_initialize(){}

//Function to determine sign of a integer variable, returns bool
template <typename T> int sgn(T val){
    return (T(0) < val) - (val < T(0));
}

void serialRead(void* params){
    vexGenericSerialEnable(SERIALPORT - 1, 0);
    vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    pros::delay(10);
    pros::screen::set_pen(COLOR_BLUE);
    double distX, distY = 0;
    while(true){
        uint8_t buffer[256];
        int bufLength = 256;
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
        if(nRead >= 0){
            std::stringstream dataStream("");
            bool recordOpticalX, recordOpticalY = false;
            for(int i=0;i<nRead;i++){
                char thisDigit = (char)buffer[i];
                if(thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X'||  thisDigit == 'C'||  thisDigit == 'Y'){
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                if(thisDigit == 'C'){
                    recordOpticalX = false;
                    dataStream >> distX;
                    pros::lcd::print(1, "Optical Flow:");
                    pros::lcd::print(2, "distX: %.2lf", distX/100);
                    dataStream.str(std::string());
                }
                if(thisDigit == 'D'){
                    recordOpticalY = false;
                    dataStream >> distY;
                    global_distY = distY/100;
                    pros::lcd::print(3, "distY: %.2lf", distY/100);
                    dataStream.str(std::string());
                }
                if (recordOpticalX) dataStream << (char)buffer[i];
                if (recordOpticalY) dataStream << (char)buffer[i];
                if (thisDigit == 'X') recordOpticalX = true;
                if (thisDigit == 'Y') recordOpticalY = true;
            }
        }
        pros::Task::delay(25);
    }
}

void brake(){
    luA.brake();
    ruA.brake();
    luB.brake();
    ruB.brake();
    llA.brake();
    rlA.brake();
    llB.brake();
    rlB.brake();
    pros::delay(1);
}

//Sets a hard capped limit for motor RPM
double bound_value(double value){
    if (value > MAX_RPM) return MAX_RPM;
    if (value < -MAX_RPM) return -MAX_RPM;
    return value;
}

//Converts rotational sensor readings into degrees and bounds it between -180 to 180
double getNormalizedSensorAngle(pros::Rotation &sensor){
    double angle = sensor.get_angle() / 100.0; //Convert from centidegrees to degrees

    if (angle < -180)
        angle += 360;
    else if (angle > 180)
        angle -= 360;

    return angle;
}

double closestAngle(double current_angle, double target_angle)
{
    //dir gives the angle from -180 to 180 degrees for the latest wheel angle
  double dir = std::fmod(current_angle, 360.0) - std::fmod(target_angle, 360.0);

  if (abs(dir) > 180.0)
    dir = -(sgn(dir) * 360.0) + dir; //flagged, this if statement is just a redundant implementation of wrapAngle(dir)

  return dir;
}

double wrapAngle(double angle){
    if (angle > 180.0)
        return angle -= 360.0;
    else if (angle < -180.0)
        return angle += 360.0;
    else
        return angle;
}

vector3D normalizeJoystick(int x_in, int y_in){
    double angle = atan2(y_in,x_in) * TO_DEGREES;
    double scaleLength;
    double magnitude;
    double length = sqrt(x_in*x_in*1.0+y_in*y_in*1.0);
    //use CSC or SEC as required
    if((angle>45.0  && angle< 135.0) || (angle<-45.0 && angle> -135.0)){
        scaleLength = 1/sin(angle) * 127.0;
    }else{
        scaleLength = 1/cos(angle) * 127.0;
    }
    scaleLength = scaleLength-DEADBAND;
    magnitude = length-DEADBAND/scaleLength-DEADBAND;
    vector3D out;
    out.load(magnitude*cos(angle*TO_RADIANS),magnitude*sin(angle*TO_RADIANS),0.0);
    return out;
}

vector3D normalizeRotation(int x_in){
    double scaleLength = 127.0-DEADBAND;
    double value = (abs(x_in)-DEADBAND)/scaleLength;
    if(x_in<0){
        value = value * -1.0;
    }
    vector3D out;
    out.load(0.0,0.0,value);
    return out;
}

void moveBase(){
    int v_right_rpm;
    int v_left_rpm;
    double theta_div;
    double v_right_magnitude;
    double v_right_direction;
    double v_left_direction;
    double v_right_velocity;
    double v_left_velocity;
    double left_angle;
    double right_angle;

    double l_error = 0.0;
    double r_error = 0.0;
    double l_p_error;
    double r_p_error;
    double l_int_error = 0.0;
    double r_int_error = 0.0;

    int lu;
    int ll;
    int ru;
    int rl;

    vector3D L2I_pos(WHEEL_BASE_RADIUS,0.0,0.0);
    while(true){
        left_angle = getNormalizedSensorAngle(left_rotation_sensor)*TO_RADIANS;
        right_angle = getNormalizedSensorAngle(right_rotation_sensor)*TO_RADIANS;

        target_v = normalizeJoystick(leftX, leftY).scalar(MAX_SPEED);
        target_r = normalizeRotation(rightX).scalar(MAX_ANGULAR);
        pros::lcd::print(0, "target_v %.1lf, %.1lf", target_v.x, target_v.y);
        pros::lcd::print(1, "target_r %.1lf", target_r.z);
        temp = ((target_v^L2I_pos)-target_r).scalar(0.5); // ^ is the cross product operator
        
        theta = atan2(target_v.y, target_v.x); // angle between direction vector and robot right, rads
        theta_div = fabs(target_r.z)*fabs(cos(theta))*THETA_MAX; //cosine map for angle deviation magnitude, degrees
        v_right_magnitude = (1/(WHEEL_BASE_RADIUS*sin(theta)))*temp.norm(); // v_right magnitude
        v_right_direction = (target_r.z>0)? theta+theta_div:theta-theta_div; //direction 
        v_right.load(v_right_magnitude*cos(v_right_direction), v_right_magnitude*sin(v_right_direction), 0.0);
        v_left = target_v - v_right;

        v_right_velocity = SPEED_TO_RPM* TRANSLATE_RATIO* v_right.norm();
        v_left_velocity = SPEED_TO_RPM* TRANSLATE_RATIO* v_left.norm();
        v_left_direction = atan2(v_left.y,v_left.x);

        pros::lcd::print(2, "%.1f, %.1f, %.1f", v_left_direction, left_angle, v_left_velocity);
        pros::lcd::print(3, "%.1f, %.1f, %.1f", v_right_direction, right_angle, v_right_velocity);

        l_p_error = l_error;
        r_p_error = r_error;
        l_error = v_left_direction-left_angle;
        r_error = v_right_direction-right_angle;
        l_int_error += l_error;
        r_int_error += r_error;

        lu = v_left_velocity + (kP*(l_error)+kI*(l_int_error)+kD*(l_p_error-l_error));
        ll = v_left_velocity - (kP*(l_error)+kI*(l_int_error)+kD*(l_p_error-l_error));
        ru = v_left_velocity + (kP*(r_error)+kI*(r_int_error)+kD*(r_p_error-r_error));
        rl = v_left_velocity - (kP*(r_error)+kI*(r_int_error)+kD*(r_p_error-r_error));;
        
        luA.move_velocity(lu);
        luB.move_velocity(lu);

        llA.move_velocity(ll);
        llB.move_velocity(ll);

        ruA.move_velocity(ru);
        ruB.move_velocity(ru);

        rlA.move_velocity(rl);
        rlB.move_velocity(rl);
        pros::Task::delay(4);
    }
}

void initialize(){
  pros::lcd::initialize();
  luA.set_brake_mode(MOTOR_BRAKE_BRAKE);
    luB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    llA.set_brake_mode(MOTOR_BRAKE_BRAKE);
    llB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    ruA.set_brake_mode(MOTOR_BRAKE_BRAKE);
    ruB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rlA.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rlB.set_brake_mode(MOTOR_BRAKE_BRAKE);
    liftL.set_brake_mode(MOTOR_BRAKE_HOLD);
    liftR.set_brake_mode(MOTOR_BRAKE_HOLD);

  while(!left_rotation_sensor.reset());
    while(!right_rotation_sensor.reset());

  left_rotation_sensor.set_data_rate(5);
    right_rotation_sensor.set_data_rate(5);

  left_rotation_sensor.set_position(0);
    right_rotation_sensor.set_position(0);

  imu.reset(true);
    imu.set_data_rate(5);

    lifter.calibrate();
    pros::Task move_base(moveBase);
    pros::Task serial_read(serialRead);

  master.clear();
}

void opcontrol(){
  while(true){
    leftX = master.get_analog(ANALOG_LEFT_X);
        leftY = master.get_analog(ANALOG_LEFT_Y);
        rightX = master.get_analog(ANALOG_RIGHT_X);

    pros::delay(5);
  }
}