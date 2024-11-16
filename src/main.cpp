#include "main.h"

void disabled(){}
void competition_initialize(){}
/*
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
            for(int i = 0;i < nRead; i++){
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
}*/

void brake(){ //brakes all base motors
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

void tareBaseMotorEncoderPositions() //tares all base motor encoder positions
{
    luA.tare_position();
    ruA.tare_position();
    luB.tare_position();
    ruB.tare_position();
    llA.tare_position();
    rlA.tare_position();
    llB.tare_position();
    rlB.tare_position();
    pros::delay(1);
}

void clampVoltage(double lu, double ll, double ru, double rl)
{
    //if any of lu, ll, ru or rl are too big, we need to scale them, and we must scale them all by the same amount so we dont throw off the proportions
    if(fabs(lu) > MAX_VOLTAGE || fabs(ll) > MAX_VOLTAGE || fabs(ru) > MAX_VOLTAGE || fabs(rl) > MAX_VOLTAGE)
    {
        //figure out which of lu, ll, ru or rl has the largest magnitude
        double max = fabs(lu);
        if(max < fabs(ll))
            max = fabs(ll);
        if(max < fabs(ru))
            max = fabs(ru);
        if(max < fabs(rl))
            max = fabs(rl);
        double VoltageScalingFactor = max / MAX_VOLTAGE; //this will definitely be positive, hence it wont change the sign of lu, ll, ru or rl.
        lu = lu / VoltageScalingFactor;
        ll = ll / VoltageScalingFactor;
        ru = ru / VoltageScalingFactor;
        rl = rl / VoltageScalingFactor;
    }
}
double wrapAngle(double angle){ //forces the angle to be within the -180 < angle < 180 range
    if (angle > 180.0)
        while (angle > 180.0)
            angle -= 360.0;
    else if (angle < -180.0)
        while (angle< -180.0)
            angle += 360.0;  
    return angle;
}

double getNormalizedSensorAngle(pros::Rotation &sensor){ //Converts rotational sensor readings into degrees and bounds it between -180 to 180
    double angle = sensor.get_angle() / 100.0; //Convert from centidegrees to degrees
    return wrapAngle(angle); //forces the angle to be within the -180 < angle < 180 range
}

vector3D normalizeJoystick(int x_in, int y_in){ //convert translation joystick input to polar vector
    double angle = atan2(y_in, x_in) * TO_DEGREES; //angle of translation polar vector. Note that atan2 automatically considers the sign to deal with the trigonometry quadrant
    double scaleLength;
    double magnitude; //magnitude of translation polar vector
    double length = sqrt(x_in * x_in + y_in * y_in);
    vector3D out;
    if(length < DEADBAND){ //if the joystick is too close to the origin, dont bother moving (this is to correct for stick drift, where the joystick doesnt default to the 0,0 position due to physical damage)
        out.load(0.0, 0.0, 0.0); //assign zero values to the xyz attributes of the vector3D named "out"
        return out;
    }
    //forcing the joystick output to be a circle not the square bounding box of the joystick
    //for any radial line of the circle, we find its length from the deadband radius to the radius of the circle of the joystick, then map the speed from 0 to 1 of that length
    //this means that in any direction, we can reach our full range of speeds without being limited
    //use CSC or SEC as required
    if((angle > 45.0  && angle < 135.0) || (angle < -45.0 && angle > -135.0))
        scaleLength = 127.0 / sin(angle * TO_RADIANS);
    else
        scaleLength = 127.0 / cos(angle * TO_RADIANS);
    
    scaleLength = fabs(scaleLength) - DEADBAND; //force scaleLength to be positive and subtract deadband
    magnitude = (length - DEADBAND) / scaleLength; //find magnitude of translation vector and scale it down (note that magnitude will always be positive)
    
    out.load(magnitude * cos(angle * TO_RADIANS), magnitude * sin(angle * TO_RADIANS), 0.0); //assign values to the xyz attributes of the vector3D named "out"
    return out;
}

vector3D normalizeRotation(int x_in){ //get rotation speed from rotation joystick
    vector3D out;
    double scaleLength = 127.0 - DEADBAND;
    if(abs(x_in) < DEADBAND){ //if the joystick is too close to the origin, dont bother moving (this is to correct for stick drift, where the joystick doesnt default to the 0,0 position due to physical damage)
        out.load(0.0, 0.0, 0.0); //assign values to the xyz attributes of the vector3D named "out"
        return out;
    }
    double value = (abs(x_in) - DEADBAND) / scaleLength; //find magnitude of rotation and scale it down
    if(x_in < 0){
        value = value * -1.0;
    }
    //this is SUPPOSED to be a vector, its not wrong
    //both normaliseRotation and normaliseJoystick return a vector for standardisation. This is intended behaviour.
    out.load(0.0, 0.0, value); //assign values to the xyz attributes of the vector3D named "out"
    return -out;
}


double angle(vector3D v1, vector3D v2){
    double dot = v1 * v2;
    double det = v1.x * v2.y - v1.y * v2.x;
    return -atan2(det, dot); //atan2 automatically considers the sign to deal with the trigonometry quadrant
}

double max(double a, double b) { //returns the larger of two doubles
    return (a > b)? a : b;
}

double min(double a, double b) { //returns the smaller of two doubles
    return (a < b)? a : b;
}

void moveBase(){
    double v_right_velocity, v_left_velocity; //numerical wheel rpm for each wheel
    double left_angle, right_angle; //numerical angle for each wheel in radians from -pi to pi
    //angular v target for the robot, its the perpendicular vector that is added/subtracted from each side. later it gets automatically scaled for the base width changing by definition of cross product
    vector3D rotational_v_vector;
    vector3D current_left_vector, current_right_vector; //vector representation of each wheel velocity
    //steering angle error
    double l_error = 0.0;
    double r_error = 0.0;
    //actual velocity as measured
    double current_l_velocity = 0.0;
    double current_r_velocity = 0.0;
    //error in magnitudes from expected to actual v_right_velocity and v_left_velocity
    double current_l_tl_error = 0.0;
    double current_r_tl_error = 0.0;
    //power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    //power output for translation component of pid
    double l_velocity_pid = 0.0;
    double r_velocity_pid = 0.0;
    //scaling down the power depending on how wrong the wheel aiming angle is
    //limited by base_v = 0.7 in definitions.h
    double lscale = 0;
    double rscale = 0;
    double current_angular = 0.0; //current angular v of the robot
    vector3D current_tl_velocity(0, 0, 0); //current translational velocity vector of the robot
    vector3D prev_target_v(0, 0, 0); //previous target translation vector
    vector3D prev_target_r(0, 0, 0); //previous target rotation vector
    //feed forward for pid
    //differentiate the control input (the joystick) to get rate of change of joystick
    //if rate of change is huge, it causes the pid controller to return an abnormally large overcompensation
    //this is done by artificially increasing the error amount when detecting large changes in input, reducing the time taken for the pid to converge
    vector3D v_fterm(0, 0, 0);
    vector3D r_fterm(0, 0, 0);
    //placeholder values for x and y components of current translational v of the robot
    double average_x_v = 0;
    double average_y_v = 0;

    uint64_t micros_now = -1;
    
    uint64_t micros_prev = pros::micros(); //pros::micros returns the number of microseconds that have passed since the program started
    uint64_t dt = -1; //f term for feed forward differentiation for rate of change 

    int32_t lu; //voltage variables for the four motor pairs of the base
    int32_t ll;
    int32_t ru;
    int32_t rl;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    PID left_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    PID right_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    
    //reference vector that points to the left with a magnitude of WHEEL_BASE_RADIUS
    vector3D L2I_pos(WHEEL_BASE_RADIUS, 0.0, 0.0);
    while(true){
        //get the current pivot angles of the left and right wheels in radians
        //subtract 90 degrees because the angle zero is defined as the positive x axis in the spline math but we want the zero angle to be defined as the wheels pointing to the front of the robot
        left_angle = getNormalizedSensorAngle(left_rotation_sensor) - 90.0 * TO_RADIANS; //note that the function getNormalizedSensorAngle already implements wrapAngle to bound the angle between -180 and 180 degrees
        right_angle = getNormalizedSensorAngle(right_rotation_sensor) - 90.0 * TO_RADIANS;
        current_left_vector = vector3D(cos(left_angle),sin(left_angle),0.0);
        current_right_vector = vector3D(cos(right_angle),sin(right_angle),0.0);

        //finding the current rotational velocity vectors of the left and right wheels
        //we have to divide by four because the velocity of the wheel is the average of the velocities of the four motors for that wheel
        //we cannot just take one motor for each gear of each wheel, since each gear is powered by two motors which could be running at marginally different speeds
        //by taking readings from all four motors of each wheel, we get more accurate results
        //just trust
        current_l_velocity = ((luA.get_actual_velocity()+luB.get_actual_velocity()+llA.get_actual_velocity()+llB.get_actual_velocity())/4.0);
        current_r_velocity = ((ruA.get_actual_velocity()+ruB.get_actual_velocity()+rlA.get_actual_velocity()+rlB.get_actual_velocity())/4.0);
        //derivation: arc length = radius * angle
        //thus, tangential velocity = radius * angular velocity
        //thus, angular velocity = tangential velocity / radius
        current_angular = (current_l_velocity*sin(left_angle)+current_r_velocity*sin(right_angle))/(2.0*WHEEL_BASE_RADIUS); //find current angular velocity of the robot
        average_x_v = ((current_l_velocity*cos(left_angle))+(current_r_velocity*cos(right_angle)))/2.0; //velocity in x direction
        average_y_v = ((current_l_velocity*sin(left_angle))+(current_r_velocity*sin(right_angle)))/2.0; //velocity in y direction
        //current translation vector of the robot
        current_tl_velocity.load(average_x_v,average_y_v,0.0); //assign values to the xyz attributes of the vector3D named "current_tl_velocity"

        prev_target_v = target_v; //target translational v of the robot
        prev_target_r = target_r; //target rotational v of the robot
        //get the joystick inputs of the robot to find out how the robot should translate and rotate
        // TODO: switch PID to go for target angle, switch actual to use current sensor angle
        target_v = normalizeJoystick(-leftX, -leftY).scalar(MAX_SPEED);
        target_r = normalizeRotation(rightX).scalar(MAX_ANGULAR); //multiply normaliseRotation(rightX) by the scalar factor MAX_ANGULAR

        //update micros_prev and micros_now
        micros_prev = micros_now;
        micros_now = pros::micros();
        dt = micros_now-micros_prev;
        v_fterm = (target_v-prev_target_v)*(v_kF/dt);
        r_fterm = (target_r-prev_target_r)*(r_kF/dt);
        target_v = target_v + v_fterm;
        target_r = target_r + r_fterm;

        rotational_v_vector = L2I_pos^target_r; //cross product of L2I_pos (vector pointing to the left with a magnitude of WHEEL_BASE_RADIUS) and target_r
        
        v_left = target_v - rotational_v_vector;
        v_right = target_v + rotational_v_vector;

        //even though we reset these booleans every loop, it should be ok because we always reevaluate whether the wheels should reverse
        bool reverse_right = false;
        bool reverse_left = false;
        
        //evaluate if we need to reverse one or both wheels
        if (v_left * current_left_vector < 0){  // check if the angle is obtuse (note that this is a VECTOR multiplication not a SCALAR multiplication. DO NOT attempt to optimise this as v_left < 0 || current_left_vector < 0)
            v_left = -v_left; // flip the v_left vector if angle is obtuse for shorter rotation
            reverse_left = true;
        }
        if (v_right * current_right_vector < 0){  // check if the angle is obtuse
            v_right = -v_right; // flip the v_right vector if angle is obtuse for shorter rotation
            reverse_right = true;
        }
        //find current numerical rpm for each wheel
        //these brackets are needed or you get an error. nobody knows why.
        v_right_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_right*current_right_vector);
        v_left_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_left*current_left_vector);

        //the magnitude of these vectors get flipped again if v_left or v_right was flipped, so the end state velocity vector is still the same as the original, just with a shorter steering rotation
        if(reverse_left)
            v_left_velocity = -v_left_velocity;
        if(reverse_right)
            v_right_velocity = -v_right_velocity;
        
        // calculate the error angle
        l_error = angle(current_left_vector, v_left);
        r_error = angle(current_right_vector, v_right);
        if (std::isnan(l_error) || std::isnan(r_error)) {
            l_error = 0.0; 
            r_error = 0.0;
        }

        //calculate the wheel error
        current_l_tl_error = (v_left_velocity - current_l_velocity);
        current_r_tl_error = (v_right_velocity - current_r_velocity);

        //calculate the PID output
        l_velocity_pid += left_velocity_PID.step(current_l_tl_error);
        r_velocity_pid += right_velocity_PID.step(current_r_tl_error);
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);
        //tuned value, reduces power output more when the wheel is facing a more incorrect way 
        //it will scale to a minimum of 0.7 as defined by the constant base_v in definitions.h
        //at zero error its at full power and scales linearly down as error increases, at max error of pi/2 it caps at 0.7
        lscale = scale * ((1.0-base_v)*fabs((l_error))+base_v);
        rscale = scale * ((1.0-base_v)*fabs((r_error))+base_v);

        //calculate voltages required to run each motor, and scale them into the acceptable voltage range so they dont exceed max voltage
        //we have to scale the voltages because if we don't, it can happen that one or more motors dont move as fast as we expected because we ordered it to move
        //at a higher voltage than it can physically achieve, and this will throw off the proportions of velocity of the four motor pairs, and cause the robot
        //to move in unexpected ways. Scaling means that sometimes the robot moves slower than expected, but at least it moves correctly otherwise.
        lu = (int32_t)(lscale * (l_velocity_pid + l_angle_pid));//this side seems less powerful on the robot
        ll = (int32_t)(lscale * (l_velocity_pid - l_angle_pid));    
        ru = (int32_t)(rscale * (r_velocity_pid + r_angle_pid));
        rl = (int32_t)(rscale * (r_velocity_pid - r_angle_pid));
        
        clampVoltage(lu, ll, ru, rl); //ensure the voltages are within usable range

        luA.move_voltage(lu);
        luB.move_voltage(lu);

        llA.move_voltage(ll);
        llB.move_voltage(ll);

        ruA.move_voltage(ru);
        ruB.move_voltage(ru);

        rlA.move_voltage(rl);
        rlB.move_voltage(rl);
    
        pros::Task::delay(1);
    }
}

void pivotWheels(double l_target_angle, double r_target_angle, double allowed_error) //pivot the left and right wheels by a certain amount
{
    double left_angle, right_angle; //numerical angle for each wheel in radians from -pi to pi
    //power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;

    //pivot angle error
    double l_angle_error = l_target_angle;
    double r_angle_error = r_target_angle;

    int32_t lu; //voltage variables for the four motor pairs of the base
    int32_t ll;
    int32_t ru;
    int32_t rl;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    
    while(fabs(l_angle_error) > allowed_error && fabs(r_angle_error) > allowed_error){ //while we havent reached the target pivot angles
        //get the current pivot angles of the left and right wheels in radians
        //subtract 90 degrees because the angle zero is defined as the positive x axis in the spline math but we want the zero angle to be defined as the wheels pointing to the front of the robot
        left_angle = getNormalizedSensorAngle(left_rotation_sensor) - 90.0 * TO_RADIANS; //note that the function getNormalizedSensorAngle already implements wrapAngle to bound the angle between -180 and 180 degrees
        right_angle = getNormalizedSensorAngle(right_rotation_sensor) - 90.0 * TO_RADIANS;
        //update pivot angle errors
        l_angle_error = left_angle - l_target_angle
        r_angle_error = right_angle - r_target_angle;

        //calculate the PID output
        l_angle_pid = left_angle_PID.step(l_angle_error);
        r_angle_pid = right_angle_PID.step(r_angle_error);

        //calculate voltages required to run each motor, and scale them into the acceptable voltage range so they dont exceed max voltage
        //we have to scale the voltages because if we don't, it can happen that one or more motors dont move as fast as we expected because we ordered it to move
        //at a higher voltage than it can physically achieve, and this will throw off the proportions of velocity of the four motor pairs, and cause the robot
        //to move in unexpected ways. Scaling means that sometimes the robot moves slower than expected, but at least it moves correctly otherwise.
        lu = (int32_t)(l_angle_pid);//this side seems less powerful on the robot
        ll = (int32_t)(-l_angle_pid);   
        ru = (int32_t)(r_angle_pid);
        rl = (int32_t)(-r_angle_pid);
        
        clampVoltage(lu, ll, ru, rl); //ensure the voltages are within usable range

        luA.move_voltage(lu);
        luB.move_voltage(lu);

        llA.move_voltage(ll);
        llB.move_voltage(ll);

        ruA.move_voltage(ru);
        ruB.move_voltage(ru);

        rlA.move_voltage(rl);
        rlB.move_voltage(rl);
    
        pros::Task::delay(1);
    }
}

void rotateWheels(double l_distance, double r_distance, double allowed_error){ //rotate the left and right wheels by a certain amount WHILE maintaining pivot angle
    double l_distance_moved = 0.0; //distance that the left and right wheel moved
    double r_distance_moved = 0.0;

    double l_distance_error = l_distance; //distance error of the wheels
    double r_distance_error = r_distance;
    
    //pivot angles of the wheel at the start of the motion, we MUST maintain these pivot angles so the robot moves straight
    double l_angleMaintain = getNormalizedSensorAngle(left_rotation_sensor) - 90.0 * TO_RADIANS; //note that the function getNormalizedSensorAngle already implements wrapAngle to bound the angle between -180 and 180 degrees
    double r_angleMaintain = getNormalizedSensorAngle(right_rotation_sensor) - 90.0 * TO_RADIANS; //note that the function getNormalizedSensorAngle already implements wrapAngle to bound the angle between -180 and 180 degrees

    double left_angle, right_angle; //numerical angle for each wheel in radians from -pi to pi
    //steering angle error
    double l_error = 0.0;
    double r_error = 0.0;
    //power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    //power output for translation component of pid
    double l_distance_pid = 0.0;
    double r_distance_pid = 0.0;
    //scaling down the power depending on how wrong the wheel aiming angle is
    //limited by base_v = 0.7 in definitions.h
    double lscale = 0;
    double rscale = 0;

    int32_t lu; //voltage variables for the four motor pairs of the base
    int32_t ll;
    int32_t ru;
    int32_t rl;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    PID left_distance_PID(distance_kP, distance_kI, distance_kD);
    PID right_distance_PID(distance_kP, distance_kI, distance_kD);

    tareBaseMotorEncoderPositions(); //tare all base motor encoder positions

    while(fabs(l_distance_error) > allowed_error && fabs(r_distance_error) > allowed_error){ //while the left and right wheels have not moved the target distance
        //get the distance that the robot has moved since we started this function
        //we have to divide by four because the velocity of the wheel is the average of the velocities of the four motors for that wheel
        //we cannot just take one motor for each gear of each wheel, since each gear is powered by two motors which could be running at marginally different speeds
        //by taking readings from all four motors of each wheel, we get more accurate results
        l_distance_moved = ((luA.get_position()+luB.get_position()+llA.get_position()+llB.get_position())/4.0) / ticks_per_mm;
        r_distance_moved = ((ruA.get_position()+ruB.get_position()+rlA.get_position()+rlB.get_position())/4.0) / ticks_per_mm;

        l_distance_error = l_distance - l_distance_moved; //calculate the error distance
        r_distance_error = r_distance - r_distance_moved;
        
        //get the current pivot angles of the left and right wheels in radians
        //subtract 90 degrees because the angle zero is defined as the positive x axis in the spline math but we want the zero angle to be defined as the wheels pointing to the front of the robot
        left_angle = getNormalizedSensorAngle(left_rotation_sensor) - 90.0 * TO_RADIANS; //note that the function getNormalizedSensorAngle already implements wrapAngle to bound the angle between -180 and 180 degrees
        right_angle = getNormalizedSensorAngle(right_rotation_sensor) - 90.0 * TO_RADIANS;

        // calculate the error angle
        l_error = l_angleMaintain - left_angle;
        r_error = r_angleMaintain - right_angle;

        //calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);
        l_distance_pid += left_distance_PID.step(l_distance_error);
        r_distance_pid += right_distance_PID.step(r_distance_error);

        //tuned value, reduces power output more when the wheel is facing a more incorrect way 
        //it will scale to a minimum of 0.7 as defined by the constant base_v in definitions.h
        //at zero error its at full power and scales linearly down as error increases, at max error of pi/2 it caps at 0.7
        lscale = scale * ((1.0-l_distance_error)*fabs((l_error))+l_distance_error);
        rscale = scale * ((1.0-r_distance_error)*fabs((r_error))+r_distance_error);

        //calculate voltages required to run each motor, and scale them into the acceptable voltage range so they dont exceed max voltage
        //we have to scale the voltages because if we don't, it can happen that one or more motors dont move as fast as we expected because we ordered it to move
        //at a higher voltage than it can physically achieve, and this will throw off the proportions of velocity of the four motor pairs, and cause the robot
        //to move in unexpected ways. Scaling means that sometimes the robot moves slower than expected, but at least it moves correctly otherwise.
        lu = (int32_t)(lscale * (l_distance_pid + l_angle_pid));//this side seems less powerful on the robot
        ll = (int32_t)(lscale * (l_distance_pid - l_angle_pid));    
        ru = (int32_t)(rscale * (r_distance_pid + r_angle_pid));
        rl = (int32_t)(rscale * (r_distance_pid - r_angle_pid));
        
        clampVoltage(lu, ll, ru, rl); //ensure the voltages are within usable range

        luA.move_voltage(lu);
        luB.move_voltage(lu);

        llA.move_voltage(ll);
        llB.move_voltage(ll);

        ruA.move_voltage(ru);
        ruB.move_voltage(ru);

        rlA.move_voltage(rl);
        rlB.move_voltage(rl);
    
        pros::Task::delay(1);
    }
}

struct MotionStepCommand { //contains the amount that the wheels should rotate and the target angle of the wheels
    double Lmove, Lpivot, Rmove, Rpivot;
    MotionStepCommand(double Lmove, double Lpivot, double Rmove, double Rpivot)
        : Lmove(Lmove), Lpivot(Lpivot), Rmove(Rmove), Rpivot(Rpivot) {}
};
struct StepCommandList{ //contains the list of commands for the base to follow in order to produce the path, AND contains the eight hermite coefficients for the left and right wheels
    std::vector<MotionStepCommand> Steps;
    double cax, cay, cbx, cby, ccx, ccy, cdx, cdy;
};
struct Waypoint{ //stores a waypoint of the auton path
    vector3d position, velocity;
    Waypoint(vector3d position, vector3d velocity)
        : position(position), velocity(velocity) {}
}

std::vector<Waypoint> ImportWaypointConfig(String config)
{
    std::vector<Waypoint>& waypoints;
    if (config.empty())
        return;
    std::string xValue, yValue, magnitude, direction;
    size_t i = 0;
    while (i < config.length()) {
        try {
            if (config[i] == 'x') {
                i++; // skip over the x character
                while (config[i] != 'y') {
                    xValue += config[i];
                    i++;
                }
            }
            if (config[i] == 'y') {
                i++; // skip over the y character
                while (config[i] != 'v') {
                    yValue += config[i];
                    i++;
                }
            }
            if (config[i] == 'v') {
                i++; // skip over the v character
                while (config[i] != 'θ') {
                    magnitude += config[i];
                    i++;
                }
            }
            if (config[i] == 'θ') {
                i++; // skip over the θ character
                while (config[i] != '&') {
                    direction += config[i];
                    i++;
                }
            }
            if (config[i] == '&') {
                i++; // skip over the & character
                waypoints.emplace_back( //construct a new waypoint and attach it to the back of the waypoints vector
                    vector3d(std::stod(xValue) / 5, std::stod(yValue) / 5, 0),
                    vector3d(std::stod(magnitude), std::stod(direction), 0)
                );
                // reset the strings to be ready to translate the next waypoint
                xValue.clear();
                yValue.clear();
                magnitude.clear();
                direction.clear();
            }
        } catch (const std::exception&) {
            break;
        }
    }
    return waypoints;
}

// A struct to represent a single point in the lookup table
struct Point {
    double x; //u value
    double y; //angle value
};

// Function to perform linear interpolation in the orientation lookup tables
double linearInterpolate(const std::vector<Point>& points, double x) {
    // Ensure the lookup table is sorted by x
    auto compare = [](const Point& a, const Point& b) { return a.x < b.x; };
    auto it = std::lower_bound(points.begin(), points.end(), Point{x, 0.0}, compare);

    // Handle cases where x is out of range
    if (it == points.begin()) {
        return points.front().y; // Return the first y-value if x is too small
    }
    if (it == points.end()) {
        return points.back().y; // Return the last y-value if x is too large
    }

    // Find the two points for interpolation
    auto p2 = *it;
    auto p1 = *(it - 1);

    // Perform linear interpolation
    double slope = (p2.y - p1.y) / (p2.x - p1.x);
    return p1.y + slope * (x - p1.x);
}

std::vector<Point> generateLocalLookupTable(double startValue, double endValue, std::vector<Point> GlobalLookupTable, double resolution) //generates a subtable of a larger lookup table
{
    std::vector<Point> LocalLookupTable = {
        {linearInterpolate(GlobalLookupTable, startValue), 0} //input the start point of the lookup table
    }
    for(int i = 0; i < 1 - (1 / resolution); i = i + 1 / resolution) //input all the points in the middle of the lookup table
        LocalLookupTable.emplace_back(linearInterpolate(GlobalLookupTable, i + startValue), i);
    LocalLookupTable.emplace_back(linearInterpolate(GlobalLookupTable, endValue), 1); //input the end point of the lookup table
}

void GetNextStep(std::vector<MotionStepCommand>& Steps, vector3D NewRobotPosition, double NewRobotOrientation, vector3D PreviousLeftWheelPosition, vector3D PreviousRightWheelPosition) {
    //apply definition of L(t) and R(t) to get current left and right wheel position
    //left and right displacements are the relative positions of the left and right wheels relative to the robot
    vector3D left_displacement(std::sin(NewRobotOrientation) * WHEEL_BASE_RADIUS, std::cos(NewRobotOrientation) * WHEEL_BASE_RADIUS);
    vector3D right_displacement(std::sin(NewRobotOrientation) * WHEEL_BASE_RADIUS, std::cos(NewRobotOrientation) * WHEEL_BASE_RADIUS);
    vector3D NewLeftWheelPosition = NewRobotPosition + left_displacement;
    vector3D NewRightWheelPosition = NewRobotPosition + right_displacement;
    //compare new left and right wheel positions to the previous left and right wheel positions to see how each wheel should move during the next step, to get from the previous to the new state
    vector3D LeftStep = NewLeftWheelPosition - PreviousLeftWheelPosition;
    vector3D RightStep = NewRightWheelPosition - PreviousRightWheelPosition;
    Steps.push_back(MotionStepCommand(LeftStep.magnitude(), LeftStep.getAngle(), RightStep.magnitude(), RightStep.getAngle())); //encode the step information into the Steps list
}

StepCommandList GenerateHermitePath(vector3D pStart, vector3D pEnd, vector3D vStart, vector3D vEnd, float StepLength, std::vector<Point> LocalOrientationLookupTable) {
    StepCommandList StepCL; //this list will store all the step motion data that causes the robot to execute the path

    //THIS PRODUCES THE HERMITE SPLINE COEFFICIENTS. THESE ARE NOT CONSTANTS FOR YOU TO TUNE. DO NOT CHANGE THESE CONSTANTS.
    StepCL.cax = 2 * pStart.x + vStart.x - 2 * pEnd.x + vEnd.x;
    StepCL.cay = 2 * pStart.y + vStart.y - 2 * pEnd.y + vEnd.y;
    StepCL.cbx = -3 * pStart.x - 2 * vStart.x + 3 * pEnd.x - vEnd.x;
    StepCL.cby = -3 * pStart.y - 2 * vStart.y + 3 * pEnd.y - vEnd.y;
    StepCL.ccx = vStart.x;
    StepCL.ccy = vStart.y;
    StepCL.cdx = pStart.x;
    StepCL.cdy = pStart.y;

    vector3D ct[4] = { //this array of vector3D represents the coefficients of the parametric polynomial function C(t) which defines the curve of the path
        vector3D(StepCL.cax, StepCL.cay),
        vector3D(StepCL.cbx, StepCL.cby),
        vector3D(StepCL.ccx, StepCL.ccy),
        vector3D(StepCL.cdx, StepCL.cdy)
    };

    vector3D CurrentRobotPosition = pStart; //current robot position is simply the position of the robot at the start of the motion
    double CurrentRobotOrientation = vector3D(vStart).getAngle(); //angle of the robot at the start of the motion is simply the angle of the robot velocity at the start of the motion
    double mEnd = vEnd.getAngle(); //angle of the robot at the end of the motion is simply the angle of the robot velocity at the end of the motion

    //left and right displacements are the positions of the left and right wheels relative to the robot
    vector3D previous_left_displacement(std::sin(CurrentRobotOrientation) * WHEEL_BASE_RADIUS, std::cos(CurrentRobotOrientation) * WHEEL_BASE_RADIUS);
    vector3D previous_right_displacement(-std::sin(CurrentRobotOrientation) * WHEEL_BASE_RADIUS, -std::cos(CurrentRobotOrientation) * WHEEL_BASE_RADIUS);
    vector3D PreviousLeftWheelPosition = CurrentRobotPosition + previous_left_displacement;
    vector3D PreviousRightWheelPosition = CurrentRobotPosition + previous_right_displacement;
    for (float t = StepLength; t < 1; t += StepLength) { //StepLength is a value to be tuned. Smaller steps produce a more accurate motion but PWM the motors more aggressively, slowing the motion down.
        //apply C(t) equation to get CurrentRobotPosition
        CurrentRobotPosition = vector3D(
            ct[0].x * std::pow(t, 3) + ct[1].x * std::pow(t, 2) + ct[2].x * t + ct[3].x, //x polynomial of the parametric equation C(t)
            ct[0].y * std::pow(t, 3) + ct[1].y * std::pow(t, 2) + ct[2].y * t + ct[3].y //y polynomial of the parametric equation C(t)
        );
        //apply LocalOrientationLookupTable to get CurrentRobotOrientation
        CurrentRobotOrientation = linearInterpolate(LocalOrientationLookupTable, t) //orientation changes according to lookup table
        GetNextStep(StepCL.Steps, CurrentRobotPosition, CurrentRobotOrientation, PreviousLeftWheelPosition, PreviousRightWheelPosition);
    }
    return StepCL;
}

void move_auton(){ //execute full auton path
    //convert the config string into a big list of waypoints
    std::vector<Waypoint> waypoints = ImportWaypointConfig("");

    const std::vector<Point> GlobalOrientationLookupTable = { //plots orientation angle (represented as y value in the table) against u value in parameter space of spline paths (represented as x value in the table)
        {0, 0},
        {1, M_PI},
        {1, -M_PI}
    };

    // Sort GlobalOrientationLookupTable by x (in case it's not already sorted)
    std::sort(GlobalOrientationLookupTable.begin(), GlobalOrientationLookupTable.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
    });

    int waypointIndex = 0;

    //generate the lookup table of orientations local to this path, based on the global lookup table
    std::vector<Point> LocalOrientationLookupTable; //plots orientation angle against t value in parameter space of THIS PATH NOT ALL OF THEM

    while(waypointIndex < waypoints.size() - 1)
    {
        LocalOrientationLookupTable.clear();
        LocalOrientationLookupTable = generateLocalLookupTable(waypointIndex, waypointIndex + 1, GlobalOrientationLookupTable, 100);
        StepCommandList stepCommands = GenerateHermitePath( //generate the path (in the form of a list of step commands for the robot to follow) to get from the current waypoint to the next waypoint
            waypoints[waypointIndex].position, 
            waypoints[waypointIndex + 1].position, 
            waypoints[waypointIndex].velocity, 
            waypoints[waypointIndex + 1].velocity,
            0.05, //step length of the path in parameter space
            LocalOrientationLookupTable); //lookup table describing the orientation of the robot as it moves

        //execute the step command list to get from the current waypoint to the next waypoint
        int StepCommandCounter = -1; //this will keep track of which step commands have been executed and which have not
        while(StepCommandCounter < stepCommands.Steps.size() - 1){ //run until the path is fully executed
            StepCommandCounter++;
            MotionStepCommand current_command(stepCommands.Steps[StepCommandCounter]); //get the current step command

            pivotWheels(current_command.Lpivot, current_command.Rpivot, 0.1); //steer the wheels to the correct angle
            rotateWheels(current_command.Lmove, current_command.Rmove, 10); //rotate the wheels the correct distance

            pros::Task::delay(1);
        }

        WaypointIndex++;

        pros::Task::delay(1);
    }
}

void autonomous(){
    move_auton();
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

    //move_auton(vector3D(1000,1000, 0.3));

    //pros::Task move_base(moveBase);
    //pros::Task serial_read(serialRead);

    master.clear();
}

void opcontrol(){
  while(true){
    leftX = master.get_analog(ANALOG_LEFT_X);
    leftY = master.get_analog(ANALOG_LEFT_Y);
    rightX = master.get_analog(ANALOG_RIGHT_X);
    if(master.get_digital_new_press(DIGITAL_B)) autonomous();
    pros::delay(5);
  }
}