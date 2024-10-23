#include "main.h"
#include "vector.h"

void disabled(){}
void competition_initialize(){}
void autonomous(){}

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
                if(thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X' || thisDigit == 'C' || thisDigit == 'Y'){
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                if(thisDigit == 'C'){
                    recordOpticalX = false;
                    dataStream >> distX;
                    pros::screen::print(TEXT_MEDIUM, 5, "[Optical Flow]");
                    pros::screen::print(TEXT_MEDIUM, 6, "distX: %.2lf", distX);
                    dataStream.str(std::string());
                }
                if(thisDigit == 'D'){
                    recordOpticalY = false;
                    dataStream >> distY;
                    pros::screen::print(TEXT_MEDIUM, 7, "distX: %.2lf", distY);
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

double apply_deadband(double value){
    return (fabs(value) > DEADBAND) ? value : 0.0;
}

double bound_value(double value){
    if (value > MAX_RPM) return MAX_RPM;
    if (value < -MAX_RPM) return -MAX_RPM;
    return value;
}

double normalize_input(double value){
    const double RANGE = 127.0-DEADBAND;
    if(fabs(value)<DEADBAND){
        value = 0;
    }
    return (value/RANGE); //0 to 1
}

double normalize_output(double value1, double value2){

}

double getNormalizedSensorAngle(pros::Rotation &sensor){
    double angle = sensor.get_angle() / 100.0; // Convert from centidegrees to degrees
    if (angle < -180)
        angle += 360;
    else if (angle > 180)
        angle -= 360;

    return angle;
}

double getAngle(int x, int y){
    double a = std::atan2(std::abs(y), std::abs(x));
    if (x < 0){
        if (y < 0)
            return (M_PI * -1) + a;
        else if (y > 0)
            return M_PI - a;
        else
            return M_PI;
    }
    else if (x > 0){
        if (y < 0)
            return a * -1;
        else if (y > 0)
            return a;
        else
            return 0;
    }
    else{
        if (y < 0)
            return -M_PI / 2;
        else if (y > 0)
            return M_PI / 2;
        else
            return NAN; // return illegal value because a zero vector has no direction
    }
}

double closestAngle(double a, double b)
{
	double dir = std::fmod(b, 360.0) - std::fmod(a, 360.0);

	if (abs(dir) > 180.0)
		dir = -(sgn(dir) * 360.0) + dir;

	return dir;
}

double wrapAngle(double angle){
    if (angle > 180.0)
        return angle -= 360;
    else if (angle < -180.0)
        return angle += 360;
    else
        return angle;
}

void swerveTranslation(){
    float LEFT_SCALE_FACTOR = 1.0;
    float RIGHT_SCALE_FACTOR = 1.0;
    float WHEEL_BASE_RAD = 15.0;

	while(true){
        vector3D<double> move_target(leftX*LEFT_SCALE_FACTOR, leftY*LEFT_SCALE_FACTOR, 0);
        vector3D<double> L2I(WHEEL_BASE_RAD,0.0,0.0);
        vector3D<double> L2I_n(-WHEEL_BASE_RAD,0.0,0.0);
        vector3D<double> right_target;
        vector3D<double> left_target;
        vector3D<double> torque_target;
		rotational = bound_value(rightX * SCALING_FACTOR); //'torque' vector




/*
        if (fabs(leftY) < 0.1){
            wheel_target_angle += 5.0;
        }

		left_wheel_speed = translation_speed;
		right_wheel_speed = translation_speed;

		left_wheel_speed = bound_value(left_wheel_speed * SCALING_FACTOR);
		right_wheel_speed = bound_value(right_wheel_speed * SCALING_FACTOR);

		double left_sensor_angle = getNormalizedSensorAngle(left_rotation_sensor);
        double right_sensor_angle = getNormalizedSensorAngle(right_rotation_sensor);

		double setpointAngleL = closestAngle(left_sensor_angle, wheel_target_angle);
		double setpointAngleFlippedL = closestAngle(left_sensor_angle, wheel_target_angle + 180.0);

		double setpointAngleR = closestAngle(right_sensor_angle, wheel_target_angle);
		double setpointAngleFlippedR = closestAngle(right_sensor_angle, wheel_target_angle + 180.0);

		if(translation_speed > 1.0){
			if (abs(setpointAngleL) <= abs(setpointAngleFlippedL)){
                if(left_wheel_speed < 0){
                    left_wheel_speed = -left_wheel_speed;
                    isLeftFlipped = false;
                }
				target_angleL = (left_sensor_angle + setpointAngleL);
			}
			else{
                if(left_wheel_speed > 0){
                    left_wheel_speed = -left_wheel_speed;
                    isLeftFlipped = true;
                }
				target_angleL = (left_sensor_angle + setpointAngleFlippedL);
			}

			if (abs(setpointAngleR) <= abs(setpointAngleFlippedR)){
                if(right_wheel_speed < 0){
                    right_wheel_speed = -right_wheel_speed;
                    isRightFlipped = false;
                }
				target_angleR = (right_sensor_angle + setpointAngleR);
			}
			else{
                if(right_wheel_speed > 0){
                    right_wheel_speed = -right_wheel_speed;
                    isRightFlipped = false;
                }
				target_angleR = (right_sensor_angle + setpointAngleFlippedR);
			}
		}
*/
		pros::Task::delay(6);
	}
}

void setWheelAngle(){
	double left_previous_error = 0;
	double right_previous_error = 0;
	double left_integral = 0;
	double right_integral = 0;
	while(true){
		double left_current_angle = getNormalizedSensorAngle(left_rotation_sensor);
		double right_current_angle = getNormalizedSensorAngle(right_rotation_sensor);

		double left_error = wrapAngle(target_angleL - left_current_angle);
        double right_error = wrapAngle(target_angleR - right_current_angle);

		left_integral += left_error;
		right_integral += right_error;

		double left_derivative = left_error - left_previous_error;
		double right_derivative = right_error - right_previous_error;

		left_previous_error = left_error;
		right_previous_error = right_error;

		double left_motor_speed = lkP * left_error + lkI * left_integral + lkD * left_derivative;
		double right_motor_speed = rkP * right_error + rkI * right_integral + rkD * right_derivative;

		left_turn_speed = left_motor_speed;
		right_turn_speed = right_motor_speed;

		if(fabs(left_error) <= 1.0){
			left_turn_speed = 0.0;
			left_integral = 0.0;
			left_error = 0.0;
			left_derivative = 0.0;
		}
		if(fabs(right_error) <= 1.0){
			right_turn_speed = 0.0;
			right_integral = 0.0;
			right_error = 0.0;
			right_derivative = 0.0;
		}
		pros::Task::delay(6);
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

	while(!left_rotation_sensor.reset());
    while(!right_rotation_sensor.reset());

	left_rotation_sensor.set_data_rate(5);
    right_rotation_sensor.set_data_rate(5);

	left_rotation_sensor.set_position(0);
    right_rotation_sensor.set_position(0);

	imu.reset(true);
    imu.set_data_rate(5);

    lifter.calibrate();

	pros::Task swerve_translation(swerveTranslation);
    pros::Task set_wheel_angle(setWheelAngle);

	master.clear();
}

void opcontrol(){
	while(true){
		leftX = apply_deadband(master.get_analog(ANALOG_LEFT_X));
        leftY = apply_deadband(master.get_analog(ANALOG_LEFT_Y));
        rightX = apply_deadband(master.get_analog(ANALOG_RIGHT_X));

		if(master.get_digital_new_press(DIGITAL_X)) liftEnable = !liftEnable;

        luA.move_velocity(-left_wheel_speed - left_turn_speed - (isLeftFlipped ? -rotational : rotational));
        llA.move_velocity(-left_wheel_speed + left_turn_speed - (isLeftFlipped ? -rotational : rotational));
        ruA.move_velocity(right_wheel_speed - right_turn_speed - (isRightFlipped ? -rotational : rotational));
        rlA.move_velocity(right_wheel_speed + right_turn_speed - (isRightFlipped ? -rotational : rotational));
        luB.move_velocity(-left_wheel_speed - left_turn_speed - (isLeftFlipped ? -rotational : rotational));
        llB.move_velocity(-left_wheel_speed + left_turn_speed - (isLeftFlipped ? -rotational : rotational));
        ruB.move_velocity(right_wheel_speed - right_turn_speed - (isRightFlipped ? -rotational : rotational));
        rlB.move_velocity(right_wheel_speed + right_turn_speed - (isRightFlipped ? -rotational : rotational));

		pros::delay(15);
	}
}
