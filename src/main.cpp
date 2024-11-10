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
}

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

double getNormalizedSensorAngle(pros::Rotation &sensor){ //Converts rotational sensor readings into degrees and bounds it between -180 to 180
    double angle = sensor.get_angle() / 100.0; //Convert from centidegrees to degrees
    return wrapAngle(angle); //forces the angle to be within the -180 < angle < 180 range
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

vector3D normalizeJoystick(int x_in, int y_in){ //convert translation joystick input to polar vector
    double angle = atan2(y_in, x_in) * TO_DEGREES; //angle of translation polar vector
    double scaleLength;
    double magnitude; //magnitude of translation polar vector
    double length = sqrt(x_in * x_in + y_in * y_in);
    vector3D out;
    if(length < DEADBAND){ //if the joystick is too close to the origin, dont bother moving (this is to correct for stick drift, where the joystick doesnt default to the 0,0 position due to physical damage)
        out.load(0.0, 0.0, 0.0); //assign zero values to the xyz attributes of the vector3D named "out"
        return out;
    }
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
    out.load(0.0, 0.0, value); //assign values to the xyz attributes of the vector3D named "out"
    return -out;
}


double angle(vector3D v1, vector3D v2){
    double dot = v1 * v2;
    double det = v1.x * v2.y - v1.y * v2.x;
    return -atan2(det, dot);
}

double max(double a, double b) { //returns the larger of two doubles
    return (a > b)? a : b;
}

double min(double a, double b) { //returns the smaller of two doubles
    return (a < b)? a : b;
}

void moveBase(){
    double v_right_velocity;
    double v_left_velocity;
    double left_angle;
    double right_angle;
    double left_target_angle;
    double right_target_angle;
    vector3D rotational_v_vector;
    
    vector3D current_left_vector;
    vector3D current_right_vector;

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
    double lscale = 0;
    double rscale = 0;

    double current_angular = 0.0;
    vector3D current_tl_velocity(0, 0, 0);
    vector3D prev_target_v(0, 0, 0);
    vector3D prev_target_r(0, 0, 0);
    vector3D v_fterm(0, 0, 0);
    vector3D r_fterm(0, 0, 0);
    double average_x_v = 0;
    double average_y_v = 0;

    uint64_t micros_now = -1;
    
    uint64_t micros_prev = pros::micros();
    uint64_t dt = -1;

    int32_t lu;
    int32_t ll;
    int32_t ru;
    int32_t rl;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    PID left_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    PID right_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    

    vector3D L2I_pos(WHEEL_BASE_RADIUS, 0.0, 0.0);
    while(true){
        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor) - 90.0) * TO_RADIANS;
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;
        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0);

        current_l_velocity = ((luA.get_actual_velocity() + luB.get_actual_velocity() + llA.get_actual_velocity() + llB.get_actual_velocity()) / 4.0);
        current_r_velocity = ((ruA.get_actual_velocity() + ruB.get_actual_velocity() + rlA.get_actual_velocity() + rlB.get_actual_velocity()) / 4.0);

        current_angular = (current_l_velocity * sin(left_angle) + current_r_velocity * sin(right_angle)) / (2.0 * WHEEL_BASE_RADIUS);
        average_x_v = ((current_l_velocity * cos(left_angle)) + (current_r_velocity * cos(right_angle))) / 2.0;
        average_y_v = ((current_l_velocity * sin(left_angle)) + (current_r_velocity * sin(right_angle))) / 2.0;
        current_tl_velocity.load(average_x_v, average_y_v, 0.0); //assign values to the xyz attributes of the vector3D named "current_tl_velocity"

        prev_target_v = target_v;
        prev_target_r = target_r;
        // TODO: switch PID to go for target angle, switch actual to use current sensor angle
        target_v = normalizeJoystick(-leftX, -leftY).scalar(MAX_SPEED);
        target_r = normalizeRotation(rightX).scalar(MAX_ANGULAR); //multiply normaliseRotation(rightX) by the scalar factor MAX_ANGULAR

        //update micros_prev and micros_now
        micros_prev = micros_now;
        micros_now = pros::micros();

        dt = micros_now-micros_prev;
        v_fterm = (target_v - prev_target_v) * (v_kF / dt);
        r_fterm = (target_r - prev_target_r) * (r_kF / dt);
        target_v = target_v + v_fterm;
        target_r = target_r + r_fterm;
        
        /*
        if(target_r.norm()<(MAX_ANGULAR*0.05) && current_angular>(MAX_ANGULAR*0.05)){
            target_r = vector3D(0,0,-0.3*current_angular);
        }
        if(target_v.norm()<(MAX_SPEED*0.05) && current_tl_velocity.norm()>(MAX_SPEED*0.05)){
            target_v = current_tl_velocity.scalar(0.1);

        pros::lcd::print(1,"r_fterm %3.3f", r_fterm.z);
        
        pros::lcd::print(4, "la_target %3.3f", (l_error+left_angle));
        pros::lcd::print(5, "ra_target %3.3f", (r_error+right_angle));

        pros::lcd::print(6, "rot_v_y %3.8f", rotational_v_vector.y);
        pros::lcd::print(7, "rot_v_x %3.8f", rotational_v_vector.x);

        pros::lcd::print(2, "la %3.3f", left_angle);
        pros::lcd::print(3, "ra %3.3f", right_angle);
        }*/

        rotational_v_vector = L2I_pos^target_r;
        
        v_left = target_v - rotational_v_vector;
        v_right = target_v + rotational_v_vector;

        //even though we reset these booleans every loop, it should be ok because we always reevaluate whether the wheels should reverse
        bool reverse_right = false;
        bool reverse_left = false;
        
        //evaluate if we need to reverse one or both wheels
        if (v_left * current_left_vector < 0){  // check if the angle is obtuse
            v_left = -v_left; // reverse if angle is obtuse for shorter rotation
            reverse_left = true;
        }
        if (v_right * current_right_vector < 0){  // check if the angle is obtuse
            v_right = -v_right; // reverse if angle is obtuse for shorter rotation
            reverse_right = true;
        }

        v_right_velocity = SPEED_TO_RPM * TRANSLATE_RATIO * v_right * current_right_vector;
        v_left_velocity = SPEED_TO_RPM * TRANSLATE_RATIO * v_left * current_left_vector;

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

        l_velocity_pid += left_velocity_PID.step(current_l_tl_error);
        r_velocity_pid += right_velocity_PID.step(current_r_tl_error);

        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);

        lscale = scale * ((1.0 - base_v) * fabs(l_error) + base_v);
        rscale = scale * ((1.0 - base_v) * fabs(r_error) + base_v);

        //calculate voltages required to run each motor, and scale them into the acceptable voltage range so they dont exceed max voltage
        //we have to scale the voltages because if we don't, it can happen that one or more motors dont move as fast as we expected because we ordered it to move
        //at a higher voltage than it can physically achieve, and this will throw off the proportions of velocity of the four motor pairs, and cause the robot
        //to move in unexpected ways. Scaling means that sometimes the robot moves slower than expected, but at least it moves correctly otherwise.
        lu = (int32_t)(lscale * (l_velocity_pid + l_angle_pid));//this side seems less powerful on the robot
        ll = (int32_t)(lscale * (l_velocity_pid - l_angle_pid));
        ru = (int32_t)(rscale * (r_velocity_pid + r_angle_pid));
        rl = (int32_t)(rscale * (r_velocity_pid - r_angle_pid));
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

struct MotionStepCommand {
    double Lmove, Lpivot, Rmove, Rpivot;
    MotionStepCommand(double Lmove, double Lpivot, double Rmove, double Rpivot)
        : Lmove(Lmove), Lpivot(Lpivot), Rmove(Rmove), Rpivot(Rpivot) {}
};
struct StepCommandList{ //contains the list of commands for the base to follow in order to produce the path, AND contains the eight hermite coefficients for the left and right wheels
    std::vector<MotionStepCommand> Steps;
    double cax, cay, cbx, cby, ccx, ccy, cdx, cdy;
};

void GetNextStep(std::vector<MotionStepCommand>& Steps, vector3D CurrentRobotPosition, double CurrentRobotOrientation, vector3D PreviousLeftWheelPosition, vector3D PreviousRightWheelPosition) {
    //apply definition of L(t) and R(t) to get current left and right wheel position
    vector3D left_displacement(std::sin(CurrentRobotOrientation) * WHEEL_BASE_RADIUS, std::cos(CurrentRobotOrientation) * WHEEL_BASE_RADIUS);
    vector3D right_displacement(std::sin(CurrentRobotOrientation) * WHEEL_BASE_RADIUS, std::cos(CurrentRobotOrientation) * WHEEL_BASE_RADIUS);
    vector3D CurrentLeftWheelPosition = CurrentRobotPosition + left_displacement;
    vector3D CurrentRightWheelPosition = CurrentRobotPosition + right_displacement;

    //compare current left and right wheel positions to the previous left and right wheel positions to see how each wheel should move during the next step, to get from the previous to the current state
    vector3D LeftStep = CurrentLeftWheelPosition - PreviousLeftWheelPosition;
    vector3D RightStep = CurrentRightWheelPosition - PreviousRightWheelPosition;

    //encode the step information into the Steps list
    Steps.push_back(MotionStepCommand(LeftStep.magnitude(), LeftStep.getAngle(), RightStep.magnitude(), RightStep.getAngle()));
}



StepCommandList GenerateHermitePath(vector3D pStart, vector3D pEnd, vector3D vStart, vector3D vEnd, float StepLength, const double mt[5]) {
    if (sizeof(mt) / sizeof(mt[0]) != 5) //mt represents the coefficients of the polynomial function m(t) which we limit to degree 4, so array size should be exactly 5
        return {};
 
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


    vector3D ct[4] = { //ct represents the coefficients of the polynomial function C(t) which defines the curve of the path
        vector3D(StepCL.cax, StepCL.cay),
        vector3D(StepCL.cbx, StepCL.cby),
        vector3D(StepCL.ccx, StepCL.ccy),
        vector3D(StepCL.cdx, StepCL.cdy)
    };

   vector3D CurrentRobotPosition = pStart;
    double CurrentRobotOrientation = vector3D(vStart).getAngle();
    double mEnd = vEnd.getAngle();

    vector3D previous_left_displacement(std::sin(CurrentRobotOrientation) * WHEEL_BASE_RADIUS, std::cos(CurrentRobotOrientation) * WHEEL_BASE_RADIUS);
    vector3D previous_right_displacement(-std::sin(CurrentRobotOrientation) * WHEEL_BASE_RADIUS, -std::cos(CurrentRobotOrientation) * WHEEL_BASE_RADIUS);
    vector3D PreviousLeftWheelPosition = CurrentRobotPosition + previous_left_displacement;
    vector3D PreviousRightWheelPosition = CurrentRobotPosition + previous_right_displacement;
    for (float t = StepLength; t < 1; t += StepLength) {
        //apply C(t) equation to get CurrentRobotPosition
        CurrentRobotPosition = vector3D(
            ct[0].x * std::pow(t, 3) + ct[1].x * std::pow(t, 2) + ct[2].x * t + ct[3].x,
            ct[0].y * std::pow(t, 3) + ct[1].y * std::pow(t, 2) + ct[2].y * t + ct[3].y
        );
        //apply m(t) equation to get CurrentRobotOrientation
        CurrentRobotOrientation = mEnd + mt[0] * std::pow(t, 4) + mt[1] * std::pow(t, 3) + mt[2] * std::pow(t, 2) + mt[3] * t + mt[4]; //orientation changes according to m(t) function
        GetNextStep(StepCL.Steps, CurrentRobotPosition, CurrentRobotOrientation, PreviousLeftWheelPosition, PreviousRightWheelPosition);
    }
    return StepCL;
}

void move_auton(vector3D delta, vector3D velocity = vector3D(0, 0, 0)){
    int32_t lu;
    int32_t ll;
    int32_t ru;
    int32_t rl;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    PID left_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    PID right_velocity_PID(velocity_kP, velocity_kI, velocity_kD);

    double v_right_velocity;
    double v_left_velocity;
    double left_angle;
    double right_angle;
    double left_target_angle;
    double right_target_angle;
    vector3D rotational_v_vector;
    
    vector3D current_left_vector;
    vector3D current_right_vector;

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
    double lscale = 0;
    double rscale = 0;
    vector3D start_pos(0, 0, 0);
    left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor) - 90.0) * TO_RADIANS;
    right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;
    current_l_velocity = ((luA.get_actual_velocity() + luB.get_actual_velocity() + llA.get_actual_velocity() + llB.get_actual_velocity()) / 4.0);
    current_r_velocity = ((ruA.get_actual_velocity() + ruB.get_actual_velocity() + rlA.get_actual_velocity() + rlB.get_actual_velocity()) / 4.0);
    double current_angular = (current_l_velocity * sin(left_angle)+current_r_velocity * sin(right_angle)) / (2.0 * WHEEL_BASE_RADIUS);
    double average_x_v = ((current_l_velocity * cos(left_angle)) + (current_r_velocity * cos(right_angle))) / 2.0;
    double average_y_v = ((current_l_velocity * sin(left_angle)) + (current_r_velocity * sin(right_angle))) / 2.0;
    vector3D start_velocity(average_x_v, average_y_v, current_angular);
    double expected_time = 0;
    double mt[5] = {0};
    mt[3] = -delta.z;   // Set the m(t) to -m_end*t

    //finding total length of the path by splitting the path into many subdivisions of tiny length, approximating each subdivision as a straight line and adding them all up
    //the greater the number of subdivisions, the more expensive the length calculation is but the more accurate the approximation
    double length = 0;  //measured in millimeters
    double t, T;
    const int n = 10000; //number of subdivisions of the function
    StepCommandList stepCommands = GenerateHermitePath(start_pos, delta, start_velocity, velocity, (1.0 / n), mt); //generate the list of step commands for the robot to follow to produce the path
    for (int i = 1; i < n; i++) {
        t = static_cast<double>(i) / n;
        T = static_cast<double>(i - 1) / n;
        vector3D v1(stepCommands.cax * std::pow(t, 3) + stepCommands.cbx * std::pow(t, 2) + stepCommands.ccx * t + stepCommands.cdx, stepCommands.cay * std::pow(t, 3) + stepCommands.cby * std::pow(t, 2) + stepCommands.ccy * t + stepCommands.cdy);
        vector3D v2((stepCommands.cax * std::pow(T, 3) + stepCommands.cbx * std::pow(T, 2) + stepCommands.ccx * T + stepCommands.cdx), stepCommands.cay * std::pow(T, 3) + stepCommands.cby * std::pow(T, 2) + stepCommands.ccy * T + stepCommands.cdy);
        length += (v1 - v2).magnitude();
    }

    //calculate the max distance over the robot must travel while accelerating to max speed
    //derivation: v^2 = u^2 + 2as (This is one of the kinematics equations that all physical objects follow. v is final velocity, u is initial velocity, a is accel and s is the distance of the motion)
    //thus, (v^2)/a = u^2 + 2s
    //thus, assuming initial velocity is zero (assuming the worst case scenario), (v^2)/a = 2s
    double BASE_ACCEL_LENGTH = (MAX_SPEED * MAX_SPEED)/ACCEL; 
    if(length < BASE_ACCEL_LENGTH)
        expected_time = sqrt(2.0 * length / ACCEL);
    else{
        double ACCEL_TIME = 2.0 * MAX_SPEED/ACCEL;
        expected_time = ((length - BASE_ACCEL_LENGTH) / MAX_SPEED) + ACCEL_TIME;
    }
    expected_time *= 1000000;// convert from seconds to microseconds

    //implement async delay
    uint64_t start_time = pros::micros(); //get the current time (pros::micros returns a value equal to the number of microseconds that have passed since the code started running)
    uint64_t current_time = 0;
    uint64_t current_state = 0;

    
    while(true){
        current_time = pros::micros() - start_time; 
        current_state = n * current_time / expected_time;
        MotionStepCommand current_command(stepCommands.Steps[current_state]);

        //find the current state of the wheels angle (and hence find the direction of the wheels) and find the wheels current velocity
        left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor) - 90.0) * TO_RADIANS;
        right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor) - 90.0) * TO_RADIANS;
        current_left_vector = vector3D(cos(left_angle), sin(left_angle), 0.0);
        current_right_vector = vector3D(cos(right_angle), sin(right_angle), 0.0);
        current_l_velocity = ((luA.get_actual_velocity() + luB.get_actual_velocity() + llA.get_actual_velocity() + llB.get_actual_velocity()) / 4.0);
        current_r_velocity = ((ruA.get_actual_velocity() + ruB.get_actual_velocity() + rlA.get_actual_velocity() + rlB.get_actual_velocity()) / 4.0);
        
        //find the target velocity of the wheels
        v_left = vector3D(sin(current_command.Lpivot), cos(current_command.Lpivot)) * current_command.Lmove;
        v_right = vector3D(sin(current_command.Rpivot), cos(current_command.Rpivot)) * current_command.Rmove;

        //even though we reset these booleans every loop, it should be ok because we always reevaluate whether the wheels should reverse
        bool reverse_right = false;
        bool reverse_left = false;
        
        //evaluate if we need to reverse one or both wheels
        if (v_left * current_left_vector < 0){  // check if the angle is obtuse
            v_left = -v_left; // reverse if angle is obtuse for shorter rotation
            reverse_left = true;
        }
        if (v_right * current_right_vector < 0){  // check if the angle is obtuse
            v_right = -v_right; // reverse if angle is obtuse for shorter rotation
            reverse_right = true;
        }

        v_right_velocity = SPEED_TO_RPM * TRANSLATE_RATIO * v_right * current_right_vector;
        v_left_velocity = SPEED_TO_RPM * TRANSLATE_RATIO * v_left * current_left_vector;

        if(reverse_left)
            v_left_velocity = -v_left_velocity;
        if(reverse_right)
            v_right_velocity = -v_right_velocity;

        // calculate the error angle
        l_error = angle(current_left_vector, v_left);
        r_error = angle(current_right_vector, v_right);
        if (std::isnan(l_error) || std::isnan(r_error)) 
        {
            l_error = 0.0; 
            r_error = 0.0;
        }
            
        //calculate the wheel error
        current_l_tl_error = (v_left_velocity - current_l_velocity);
        current_r_tl_error = (v_right_velocity - current_r_velocity);

        l_velocity_pid += left_velocity_PID.step(current_l_tl_error);
        r_velocity_pid += right_velocity_PID.step(current_r_tl_error);

        // calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);

        lscale = scale * ((1.0 - base_v) * fabs(l_error) + base_v);
        rscale = scale * ((1.0 - base_v) * fabs(r_error) + base_v);

        lu = (int32_t)std::clamp(lscale * (l_velocity_pid + l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); //this side seems less powerful on the robot
        ll = (int32_t)std::clamp(lscale * (l_velocity_pid - l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        ru = (int32_t)std::clamp(rscale * (r_velocity_pid + r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        rl = (int32_t)std::clamp(rscale * (r_velocity_pid - r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
    

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

void autonomous(){
    move_auton(vector3D(1000,1000, 0.3));
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