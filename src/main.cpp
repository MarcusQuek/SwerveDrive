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

double wrapAngle(double angle){
    if (angle > 180.0){
        while (angle>180.0){
            angle -= 360.0;
        }
    }else if (angle < -180.0){
        while (angle< -180.0){
            angle += 360.0;
        }
    }    
    return angle;
}

double closestAngle(double angle, double target_angle){// in degrees
    if((target_angle-angle)>180.0){
        return -(360-(target_angle-angle));
    }else{
        return target_angle-angle;
    }
}


vector3D normalizeJoystick(int x_in, int y_in){
    double angle = atan2(y_in,x_in) * TO_DEGREES;
    double scaleLength;
    double magnitude;
    double length = sqrt(x_in*x_in*1.0+y_in*y_in*1.0);
    vector3D out;
    if(length<DEADBAND){
        out.load(0.0,0.0,0.0);
        return out;
    }
    //use CSC or SEC as required
    if((angle>45.0  && angle< 135.0) || (angle<-45.0 && angle> -135.0)){
        scaleLength = 127.0/sin(angle*TO_RADIANS);
    }else{
        scaleLength = 127.0/cos(angle*TO_RADIANS);
    }
    scaleLength = fabs(scaleLength)-DEADBAND;
    magnitude = (length-DEADBAND)/scaleLength;
    
    
    out.load(magnitude*cos(angle*TO_RADIANS),magnitude*sin(angle*TO_RADIANS),0.0);
    return out;
}

vector3D normalizeRotation(int x_in){
    vector3D out;
    double scaleLength = 127.0-DEADBAND;
    if(abs(x_in)<DEADBAND){
        out.load(0.0,0.0,0.0);
        return out;
    }
    double value = (abs(x_in)-DEADBAND)/scaleLength;
    if(x_in<0){
        value = value * -1.0;
    }
    out.load(0.0,0.0,value);
    return -out;
}


double angle(vector3D v1, vector3D v2){
    double dot = v1*v2;
    double det = v1.x * v2.y - v1.y * v2.x;
    return -atan2(det, dot);
}

double max(double a, double b) {
    return (a > b)? a : b;
}

double min(double a, double b) {
    return (a < b)? a : b;
}

void moveBase(){
    int v_right_rpm;
    int v_left_rpm;
    double theta_div;
    double v_right_magnitude;
    double v_left_magnitude;
    double v_right_velocity;
    double v_left_velocity;
    double left_angle;
    double right_angle;
    double left_target_angle;
    double right_target_angle;
    vector3D rotational_v_vector;
    
    vector3D current_left_vector;
    vector3D current_right_vector;
    double sin_left;
    double sin_right;
    double TOLERANCE = 5.0*TO_RADIANS;

    double l_error = 0.0;
    double r_error = 0.0;

    double current_l_velocity = 0.0;
    double current_r_velocity = 0.0;
    
    double current_l_tl_error = 0.0;
    double current_r_tl_error = 0.0;

    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;

    double l_velocity_pid = 0.0;
    double r_velocity_pid = 0.0;

    int32_t lu;
    int32_t ll;
    int32_t ru;
    int32_t rl;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    PID left_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    PID right_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    

    vector3D L2I_pos(WHEEL_BASE_RADIUS,0.0,0.0);
    while(true){

        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor)-90.0)*TO_RADIANS;
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor)-90.0)*TO_RADIANS;
        current_left_vector = vector3D(cos(left_angle),sin(left_angle),0.0);
        current_right_vector = vector3D(cos(right_angle),sin(right_angle),0.0);

        // TODO: switch PID to go for target angle, switch actual to use current sensor angle
        target_v = normalizeJoystick(leftX, leftY).scalar(MAX_SPEED);
        target_r = normalizeRotation(rightX).scalar(MAX_ANGULAR);

        rotational_v_vector = L2I_pos^target_r;
        
        v_left = target_v-rotational_v_vector;
        v_right = target_v+rotational_v_vector;

        bool reverse_right = false;
        bool reverse_left = false;
        
        // check if the angle is obtuse
        if (v_left * current_left_vector < 0){  
            // reverse if angle is obtuse for shorter rotation
            v_left = -v_left;
            reverse_left = true;
        }

        if (v_right * current_right_vector < 0){  
            // reverse if angle is obtuse for shorter rotation
            v_right = -v_right;
            reverse_right = true;
        }

        v_right_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_right*current_right_vector);
        v_left_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_left*current_left_vector);

        if(reverse_left){
            v_left_velocity = -v_left_velocity;
        }

        if(reverse_right){
            v_right_velocity = -v_right_velocity;
        }

        pros::lcd::print(4, "left reverse %d", reverse_left);
        pros::lcd::print(5, "righ reverse %d", reverse_right);
        

        // calculate the error angle
        l_error = angle(current_left_vector, v_left);
        r_error = angle(current_right_vector, v_right);
        if (std::isnan(l_error) || std::isnan(r_error)) {
            l_error = 0.0; r_error = 0.0;
        }

        //calculate the wheel error
        current_l_velocity = ((luA.get_actual_velocity()+luB.get_actual_velocity()+llA.get_actual_velocity()+llB.get_actual_velocity())/4.0);
        current_r_velocity = ((ruA.get_actual_velocity()+ruB.get_actual_velocity()+rlA.get_actual_velocity()+rlB.get_actual_velocity())/4.0);
        current_l_tl_error = (v_left_velocity-current_l_velocity);
        current_r_tl_error = (v_right_velocity-current_r_velocity);
        
        l_velocity_pid += left_velocity_PID.step(current_l_tl_error);
        r_velocity_pid += right_velocity_PID.step(current_r_tl_error);
        
        pros::lcd::print(1, "current_l  %3.2f", current_l_velocity);        
        pros::lcd::print(2, "tl_l_error %3.2f", current_l_tl_error);
        pros::lcd::print(3, "current_r  %3.2f", current_r_velocity);
        pros::lcd::print(4, "tl_r_error %3.2f", current_r_tl_error);


        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);

        double scale = 30;
        
        lu = (int32_t)std::clamp(scale * (l_velocity_pid + l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); //this side seems less powerful on the robot
        ll = (int32_t)std::clamp(scale * (l_velocity_pid - l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        ru = (int32_t)std::clamp(scale * (r_velocity_pid + r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        rl = (int32_t)std::clamp(scale * (r_velocity_pid - r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        
        //pros::lcd::print(4, "diff %6.4f, %6.4f", kD*l_p_error - kD*l_error, (kD*l_p_error - kD*l_error)/dt);
        // pros::lcd::print(6, "p %5.1f pterm %5.2f", angle_kP, angle_kP*(l_error));
        // pros::lcd::print(4, "error %10.7f", l_error);
        // pros::lcd::print(5, "prev  %10.7f", l_p_error);
        // pros::lcd::print(7, "d %8.2f dterm %10.7f", kD, l_dterm);

        

        luA.move_voltage(lu);
        luB.move_voltage(lu);

        llA.move_voltage(ll);
        llB.move_voltage(ll);

        ruA.move_voltage(ru);
        ruB.move_voltage(ru);

        rlA.move_voltage(rl);
        rlB.move_voltage(rl);

        pros::Task::delay(2);
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